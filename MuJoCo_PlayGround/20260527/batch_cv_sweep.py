from __future__ import annotations
"""
batch_cv_sweep.py  —  CV(곡률) 배치 스위프 실험
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
핵심 질문:
    알약 곡률(CV) → 점접촉 vs 면접촉 →  충격량(J = ∫F dt)에 어떤 영향?

실험 설계:
    고정: R = 6.0 mm,  AR = 1.50,  density = 1200 kg/m³
    변수: CV = 0.08 → 0.35  (10단계)
    각 CV → Rs(구면 반경) 변화:
        CV 작음 → Rs 큼  → 편평한 알약  → 면접촉
        CV 큼   → Rs 작음 → 볼록한 알약  → 점접촉

두 가지 모드로 비교:
    [모드 A] 고정 τ  : 모든 CV에 동일한 솔버 강성 (현재 모델)
    [모드 B] Hertz τ : Rs 기반으로 τ를 Hertz 접선 강성에 맞춰 보정

이론적 예측 (Hertz 탄성 충돌):
    J   ≈ 일정        (모멘텀 보존, CV 무관)
    F_max ∝ Rs^(1/5)  (편평할수록 힘 분산)
    P_max ∝ Rs^(-2/3) (볼록할수록 압력 집중)
    A_max ∝ Rs^(2/5)  (편평할수록 면적 큼)
    T_con ∝ Rs^(-1/5) (편평할수록 접촉 시간 길어짐)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import os
import sys
import csv
import math
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
plt.rcParams['font.family'] = 'Malgun Gothic'
plt.rcParams['axes.unicode_minus'] = False
import mujoco
import xml.etree.ElementTree as ET

# ── 공통 유틸 import (같은 디렉토리의 crusher_tablet_sim.py) ───────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from crusher_tablet_sim import (                            # noqa: E402
    MJCF_PATH, MJCF_DIR, STL_DIR,
    PLACE_X_MM, PLACE_Z_MM, WALL_Y_MM,
    PHASE1_STEPS, MOTOR_CTRL, MOTOR_DELAY,
    TAB_QUAT,
    SOLREF_TAU_SOFT, SOLREF_TAU_HARD,
    density_to_solref_tau,
)

# NumPy 2.0 compat: trapz → trapezoid (getattr default는 먼저 평가되므로 try/except 사용)
try:
    _np_trapz = np.trapezoid
except AttributeError:
    _np_trapz = np.trapz

# ─── 실험 파라미터 ────────────────────────────────────────────────────────────
R_MM      = 6.0
AR_VAL    = 1.50
DENSITY   = 1200.0    # kg/m³  — 고정 τ 기준값

CVS = [0.08, 0.11, 0.14, 0.17, 0.20, 0.23, 0.26, 0.29, 0.32, 0.35]

SIM_DURATION  = 12.0   # 배치용 단축 시간 [s]
STALL_TIME_S  = 1.5
STALL_VEL_THR = 0.05

# Hertz τ 보정용 물성 (대표값: 중간 경도 정제)
E_TABLET_PA = 5e9     # Young's modulus [Pa]
NU_TABLET   = 0.30
E_STAR      = E_TABLET_PA / (1.0 - NU_TABLET ** 2)   # 유효 탄성 계수
HERTZ_F0_N  = 50.0    # 운영점 힘 [N] — τ 선형화 기준
# L8_Link3_Shaft 실측 질량 (crusher_tablet_sim 주석 기준)
SLIDER_MASS_KG = 0.35

# 결과 저장
_RESULT_DIR = os.path.normpath(
    os.path.join(_HERE, "..", "Sim_result", "cv_sweep"))


# ─── 데이터 구조 ──────────────────────────────────────────────────────────────
@dataclass
class CVResult:
    cv:       float
    Rs_mm:    float
    tau:      float
    mode:     str          # "fixed" | "hertz"
    # 시계열 (numpy arrays)
    t:         np.ndarray = field(default_factory=lambda: np.array([]))
    Fy:        np.ndarray = field(default_factory=lambda: np.array([]))
    delta_m:   np.ndarray = field(default_factory=lambda: np.array([]))
    area_mm2:  np.ndarray = field(default_factory=lambda: np.array([]))
    press_MPa: np.ndarray = field(default_factory=lambda: np.array([]))
    # 스칼라 요약
    J:            float = 0.0   # 충격량  [N·s]
    Fy_max:       float = 0.0   # 최대 힘 [N]
    P_max_MPa:    float = 0.0   # 최대 압력 [MPa]
    A_max_mm2:    float = 0.0   # 최대 면적 [mm²]
    delta_max_um: float = 0.0   # 최대 관통깊이 [μm]
    T_contact_s:  float = 0.0   # 접촉 지속 시간 [s]
    ok:           bool  = False
    # 시간 기준점 (정규화용)
    t0:          float = 0.0    # Phase 2 시작 절대 시간 [s]
    t_motor_on:  float = 0.0    # 모터 ON 절대 시간 [s]


# ─── 기하 헬퍼 ────────────────────────────────────────────────────────────────
def _rs_mm(R_mm: float, cv: float) -> float:
    """biconvex 구면 반경 [mm].  Rs = (R²+cd²)/(2cd),  cd = cv×2R"""
    cd = cv * 2.0 * R_mm
    return (R_mm ** 2 + cd ** 2) / (2.0 * cd)


def _tau_hertz(Rs_m: float,
               F0_N: float = HERTZ_F0_N,
               ref_mass_kg: float = SLIDER_MASS_KG) -> float:
    """
    Hertz 접선 강성 기반 solref τ 보정.

    운영점 F0 에서 선형화:
        δ₀ = (3F₀ / (4E*√Rs))^(2/3)
        k_t = 2E*√(Rs·δ₀)          [N/m]
        τ   = √(m / k_t)            [s]
    τ 범위를 MuJoCo 안정 구간으로 클램핑.
    """
    if Rs_m < 1e-9:
        return SOLREF_TAU_HARD
    delta_0 = (3.0 * F0_N / (4.0 * E_STAR * math.sqrt(Rs_m))) ** (2.0 / 3.0)
    k_t     = 2.0 * E_STAR * math.sqrt(max(Rs_m * delta_0, 1e-20))
    tau     = math.sqrt(ref_mass_kg / k_t)
    return float(np.clip(tau, SOLREF_TAU_HARD, SOLREF_TAU_SOFT))


# ─── 모델 빌더 ────────────────────────────────────────────────────────────────
def _build_model(stl_path: str, half_th_mm: float, tau: float):
    """Crusher XML + Tablet STL → MjModel.  솔버 τ 직접 지정."""
    pos_x = PLACE_X_MM * 1e-3
    pos_z = PLACE_Z_MM * 1e-3
    pos_y = (WALL_Y_MM - half_th_mm) * 1e-3

    tree = ET.parse(MJCF_PATH)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler"); root.insert(0, compiler)
    compiler.set("meshdir", MJCF_DIR)

    for kf in root.findall("keyframe"):
        root.remove(kf)

    asset = root.find("asset")
    ET.SubElement(asset, "mesh",     {"name": "tablet_mesh",
                                      "file": "tablet.stl",
                                      "scale": ".001 .001 .001"})
    ET.SubElement(asset, "material", {"name": "tablet_mat",
                                      "rgba": ".85 .80 .72 1",
                                      "specular": ".4", "shininess": ".3"})

    dimp_max   = float(np.interp(DENSITY, [900.0, 1800.0], [0.950, 0.999]))
    solref_str = f"{tau:.6f} 1"
    solimp_str = f"0.99 {dimp_max:.4f} 0.0001"

    worldbody = root.find("worldbody")
    tab = ET.SubElement(worldbody, "body", {
        "name": "tablet", "mocap": "true",
        "pos": f"{pos_x:.6f} {pos_y:.6f} {pos_z:.6f}",
        "quat": " ".join(f"{v:.7f}" for v in TAB_QUAT),
    })
    ET.SubElement(tab, "geom", {
        "name": "tablet_geom", "type": "mesh", "mesh": "tablet_mesh",
        "material": "tablet_mat", "density": f"{DENSITY:.1f}",
        "condim": "4", "friction": ".5 .02 .01",
        "solref": solref_str, "solimp": solimp_str,
    })

    xml_str   = ET.tostring(root, encoding="unicode")
    stl_bytes = open(stl_path, "rb").read()
    model     = mujoco.MjModel.from_xml_string(
        xml_str, assets={"tablet.stl": stl_bytes})
    return model, (pos_x, pos_y, pos_z)


# ─── 접촉 데이터 수집 ────────────────────────────────────────────────────────
def _contact_info(model, data, body_id: int, Rs_m: float):
    """
    접촉 정보 수집 → (Fy_world [N], delta_max [m], A_total [m²])

    Fy : 법선력  (World Y 성분, 알약 기준 +방향 = 압축)
    delta_max : 가장 깊은 관통 깊이
    A_total   : π·Rs·Σδᵢ  (Hertz 접촉 면적 합산)
    """
    force6    = np.zeros(6)
    Fy_total  = 0.0
    delta_max = 0.0
    A_total   = 0.0

    for i in range(data.ncon):
        c    = data.contact[i]
        g1_b = model.geom_bodyid[c.geom1]
        g2_b = model.geom_bodyid[c.geom2]
        if body_id not in (g1_b, g2_b):
            continue

        # 법선력 → world frame
        frame   = c.frame.reshape(3, 3)
        mujoco.mj_contactForce(model, data, i, force6)
        f_world = frame.T @ force6[:3]
        if g2_b == body_id:
            f_world = -f_world
        Fy_total += f_world[1]

        # Hertz 접촉 면적 (δ 기반)
        delta_i   = abs(min(float(c.dist), 0.0))
        delta_max = max(delta_max, delta_i)
        A_total  += math.pi * Rs_m * delta_i

    return Fy_total, delta_max, A_total    # [N], [m], [m²]


# ─── 단일 CV 헤드리스 시뮬레이션 ─────────────────────────────────────────────
def run_one_cv(cv: float, tau: float, mode: str, verbose: bool = True) -> CVResult:
    """
    지정된 CV와 τ로 헤드리스 시뮬레이션 실행.
    Viewer 없음 → 배치 실험에 최적화.
    """
    cd      = cv * 2.0 * R_MM
    th_mm   = R_MM * 0.20 + 2.0 * cd
    half_th = th_mm / 2.0
    Rs_mm   = _rs_mm(R_MM, cv)
    Rs_m    = Rs_mm * 1e-3

    res = CVResult(cv=cv, Rs_mm=Rs_mm, tau=tau, mode=mode)

    fname = f"tablet_R{R_MM:.1f}_AR{AR_VAL:.2f}_CV{cv:.2f}.stl"
    fpath = os.path.join(STL_DIR, fname)
    if not os.path.exists(fpath):
        print(f"  [SKIP] STL 없음: {fname}")
        return res

    if verbose:
        print(f"  CV={cv:.2f}  Rs={Rs_mm:5.1f}mm  τ={tau:.5f}s  [{mode}]", end="  ")

    # 모델 구성
    model, (px, py, pz) = _build_model(fpath, half_th, tau)
    data = mujoco.MjData(model)

    # ID 조회
    crank_jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                    "L3_Bevel_GearBox_1_L4_Shaft_1")
    act_crank  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                    "Motor1_crank")
    b_tablet   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,   "tablet")
    eq_lock_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY,"lock_crank")

    crank_qadr = model.jnt_qposadr[crank_jid]
    crank_vadr = model.jnt_dofadr[crank_jid]
    mocap_id   = model.body_mocapid[b_tablet]
    _dt        = float(model.opt.timestep)

    # 초기 상태
    data.qpos[crank_qadr]     = -np.pi / 2
    data.qvel[:]              = 0.0
    data.mocap_pos[mocap_id]  = [px, py, pz]
    data.mocap_quat[mocap_id] = TAB_QUAT
    mujoco.mj_forward(model, data)

    # Phase 1: 안정화 (뷰어 없음)
    for _ in range(PHASE1_STEPS):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    # Phase 2: 헤드리스 메인 루프
    stall_win = max(1, int(round(STALL_TIME_S / _dt)))
    stall_buf = deque(maxlen=stall_win)
    motor_on  = False
    motor_dir = 0
    t0        = data.time
    t_motor_on_abs = t0 + MOTOR_DELAY   # 예상 모터 ON 시각 (기록용)

    res.t0         = t0
    res.t_motor_on = t_motor_on_abs

    t_log     = []
    Fy_log    = []
    delta_log = []
    area_log  = []   # [mm²]

    while data.time < (t0 + SIM_DURATION):
        # 모터 ON
        if not motor_on and (data.time - t0) >= MOTOR_DELAY:
            data.eq_active[eq_lock_id] = 0
            motor_dir = 1
            data.ctrl[act_crank] = motor_dir * MOTOR_CTRL
            motor_on  = True
            stall_buf.clear()

        # Stall 감지 → 방향 전환
        if motor_on:
            omega = abs(data.qvel[crank_vadr])
            stall_buf.append(omega < STALL_VEL_THR)
            if len(stall_buf) == stall_win and all(stall_buf):
                motor_dir = -motor_dir
                data.ctrl[act_crank] = motor_dir * MOTOR_CTRL
                stall_buf.clear()

        mujoco.mj_step(model, data)

        Fy, delta, area_m2 = _contact_info(model, data, b_tablet, Rs_m)
        t_log.append(data.time)
        Fy_log.append(Fy)
        delta_log.append(delta)
        area_log.append(area_m2 * 1e6)    # m² → mm²

    # ── numpy 변환 + 파생 계산 ────────────────────────────────────
    t        = np.array(t_log)
    Fy       = np.array(Fy_log)
    delta    = np.array(delta_log)
    area_mm2 = np.array(area_log)
    area_m2  = area_mm2 * 1e-6

    # Hertz 최대 압력: P_max = (3/2) × F / A
    P_Pa     = np.where(area_m2 > 1e-18,
                        1.5 * np.abs(Fy) / area_m2,
                        0.0)
    P_MPa    = P_Pa * 1e-6

    Fy_pos   = np.maximum(0.0, Fy)
    J        = float(_np_trapz(Fy_pos, t))

    in_con   = delta > 1e-9

    res.t          = t
    res.Fy         = Fy
    res.delta_m    = delta
    res.area_mm2   = area_mm2
    res.press_MPa  = P_MPa
    res.J          = J
    res.Fy_max     = float(Fy.max()) if len(Fy) > 0 else 0.0
    res.P_max_MPa  = float(P_MPa.max()) if len(P_MPa) > 0 else 0.0
    res.A_max_mm2  = float(area_mm2.max()) if len(area_mm2) > 0 else 0.0
    res.delta_max_um = float(delta.max() * 1e6) if len(delta) > 0 else 0.0
    res.T_contact_s  = float(np.sum(in_con) * _dt)
    res.ok           = True

    if verbose:
        print(f"J={J:.4f} N·s  F={res.Fy_max:.2f} N  "
              f"P={res.P_max_MPa:.2f} MPa  "
              f"A={res.A_max_mm2:.4f} mm²  "
              f"δ={res.delta_max_um:.2f} μm")
    return res


# ─── 개별 CV PNG 저장 ────────────────────────────────────────────────────────
def plot_individual_cvs(results_A: list[CVResult],
                        results_B: list[CVResult],
                        save_dir: str,
                        ts: str) -> None:
    """
    CV 하나당 PNG 1개 생성 → 총 10개 저장.

    각 PNG 구성 (3단 subplot):
        ① 접촉력  F_Y [N]      — 모드A(파랑) + 모드B(빨강)
        ② 접촉 압력 P [MPa]    — 모드A + 모드B
        ③ 접촉 면적 A [mm²]    — 모드A + 모드B

    X축 정규화:
        t_rel = t - t_motor_on   (모터 ON 기준 0초)
        모든 10개 플롯이 동일한 X범위 → 직접 비교 가능

    Y축 고정:
        전체 CV 기준 global min/max → 10개 플롯 동일 스케일
    """
    # ── Global Y 범위 계산 (공정 비교를 위해 10개 동일) ─────────────────
    def _safe_max(results, attr):
        vals = [getattr(r, attr) for r in results if r.ok]
        return max(vals) if vals else 1.0

    Fy_global_max  = max(_safe_max(results_A, 'Fy_max'),
                         _safe_max(results_B, 'Fy_max')) * 1.15
    Fy_global_min  = min(
        min((r.Fy.min() for r in results_A if r.ok and len(r.Fy) > 0), default=0),
        min((r.Fy.min() for r in results_B if r.ok and len(r.Fy) > 0), default=0),
    ) * 1.1

    P_global_max   = max(_safe_max(results_A, 'P_max_MPa'),
                         _safe_max(results_B, 'P_max_MPa')) * 1.15
    A_global_max   = max(_safe_max(results_A, 'A_max_mm2'),
                         _safe_max(results_B, 'A_max_mm2')) * 1.15

    # X범위: 0 ~ SIM_DURATION (모터 ON 기준)
    x_max = SIM_DURATION

    saved = []
    for rA, rB in zip(results_A, results_B):
        if not rA.ok and not rB.ok:
            continue

        cv    = rA.cv
        Rs_mm = rA.Rs_mm

        fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        fig.subplots_adjust(hspace=0.08, top=0.88, bottom=0.09,
                            left=0.10, right=0.97)

        # 접촉 유형 레이블
        if cv <= 0.12:
            contact_type = "면접촉 (flat-like)"
            type_color   = "#1a7abf"
        elif cv >= 0.30:
            contact_type = "점접촉 (point-like)"
            type_color   = "#c0392b"
        else:
            contact_type = "중간 (semi-Hertz)"
            type_color   = "#8e44ad"

        fig.suptitle(
            f"CV = {cv:.2f}  |  Rs = {Rs_mm:.1f} mm  |  {contact_type}",
            fontsize=13, fontweight='bold', color=type_color,
        )

        plot_data = [
            (rA.Fy,        rB.Fy,        "F_Y [N]",     Fy_global_min, Fy_global_max),
            (rA.press_MPa, rB.press_MPa, "P_max [MPa]", 0,             P_global_max),
            (rA.area_mm2,  rB.area_mm2,  "Area [mm²]",  0,             A_global_max),
        ]

        for i, (yA, yB, ylabel, ymin, ymax) in enumerate(plot_data):
            ax = axes[i]

            # 모터 ON 기준 정규화 시간
            if rA.ok and len(rA.t) > 0:
                tA_rel = rA.t - rA.t_motor_on
                ax.plot(tA_rel, yA, color='#1f77b4', lw=1.4,
                        label=f"A: 고정τ={rA.tau*1e3:.2f}ms", zorder=3)

            if rB.ok and len(rB.t) > 0:
                tB_rel = rB.t - rB.t_motor_on
                ax.plot(tB_rel, yB, color='#d62728', lw=1.4,
                        ls='--', label=f"B: Hertzτ={rB.tau*1e3:.2f}ms", zorder=3)

            ax.axhline(0, color='k', lw=0.5, ls=':')
            ax.axvline(0, color='gray', lw=0.8, ls='--', alpha=0.6,
                       label='모터 ON')
            ax.set_ylabel(ylabel, fontsize=10)
            ax.set_xlim(-MOTOR_DELAY * 0.05, x_max - MOTOR_DELAY)
            ax.set_ylim(ymin - abs(ymin) * 0.02, ymax)
            ax.grid(True, alpha=0.25)
            if i == 0:
                ax.legend(fontsize=8, loc='upper right', ncol=3)

        axes[2].set_xlabel("Time since motor ON [s]", fontsize=10)

        # ── 우측 정보 박스 ─────────────────────────────────────────────
        info = (
            f"Rs = {Rs_mm:.1f} mm\n"
            f"──── 모드 A (고정τ) ────\n"
            f"  J    = {rA.J:.4f} N·s\n"
            f"  F_max= {rA.Fy_max:.2f} N\n"
            f"  P_max= {rA.P_max_MPa:.3f} MPa\n"
            f"  A_max= {rA.A_max_mm2:.4f} mm²\n"
            f"──── 모드 B (Hertz) ────\n"
            f"  J    = {rB.J:.4f} N·s\n"
            f"  F_max= {rB.Fy_max:.2f} N\n"
            f"  P_max= {rB.P_max_MPa:.3f} MPa\n"
            f"  A_max= {rB.A_max_mm2:.4f} mm²\n"
        )
        fig.text(0.975, 0.5, info,
                 transform=fig.transFigure,
                 fontsize=7.5, family='monospace',
                 va='center', ha='right',
                 bbox=dict(boxstyle='round,pad=0.4',
                           facecolor='#f0f0f0', alpha=0.85, lw=0.5))

        fname = f"cv_sweep_{ts}_CV{cv:.2f}.png"
        fpath = os.path.join(save_dir, fname)
        fig.savefig(fpath, dpi=150, bbox_inches='tight')
        plt.close(fig)
        saved.append(fname)
        print(f"  ✔ {fname}")

    print(f"\n  총 {len(saved)}개 PNG 저장 → {save_dir}")


# ─── 시각화 ──────────────────────────────────────────────────────────────────
def _colormap(n: int):
    return [cm.coolwarm(i / max(n - 1, 1)) for i in range(n)]


def plot_sweep(results_A: list[CVResult],
               results_B: list[CVResult]) -> list[plt.Figure]:
    figs   = []
    colors = _colormap(len(results_A))
    CVS_np = np.array([r.cv   for r in results_A])
    Rs_np  = np.array([r.Rs_mm for r in results_A])

    # ── Fig 1: F(t) 오버레이  (모드 A / B 나란히) ─────────────────
    fig1, axes = plt.subplots(1, 2, figsize=(16, 5), sharey=True)
    for results, ax, title in [
        (results_A, axes[0], "모드 A  —  고정 τ (밀도 기반)"),
        (results_B, axes[1], "모드 B  —  Hertz 보정 τ"),
    ]:
        for r, c in zip(results, colors):
            if not r.ok: continue
            ax.plot(r.t, r.Fy, color=c, lw=1.1, alpha=0.85,
                    label=f"CV={r.cv:.2f} Rs={r.Rs_mm:.0f}mm")
        ax.axhline(0, color='k', lw=0.5, ls='--')
        ax.set_title(title, fontsize=10, fontweight='bold')
        ax.set_xlabel("Time [s]"); ax.set_ylabel("F_Y [N]")
        ax.legend(fontsize=6.5, ncol=2, loc='upper right')
        ax.grid(True, alpha=0.3)

    sm = plt.cm.ScalarMappable(cmap='coolwarm',
                               norm=plt.Normalize(CVS_np.min(), CVS_np.max()))
    sm.set_array([])
    fig1.colorbar(sm, ax=axes[1], label='CV', shrink=0.85)
    fig1.suptitle(
        f"법선 접촉력 F_Y  |  R={R_MM}mm AR={AR_VAL}  "
        f"← 파란=편평(면접촉)   볼록(점접촉)=빨간 →",
        fontsize=11, fontweight='bold')
    fig1.tight_layout()
    figs.append(fig1)

    # ── Fig 2: P(t)·A(t) 시계열 (모드 A 기준) ─────────────────────
    fig2, axes2 = plt.subplots(2, 1, figsize=(13, 7), sharex=False)
    for r, c in zip(results_A, colors):
        if not r.ok: continue
        axes2[0].plot(r.t, r.press_MPa, color=c, lw=1.0, alpha=0.85,
                      label=f"CV={r.cv:.2f}")
        axes2[1].plot(r.t, r.area_mm2,  color=c, lw=1.0, alpha=0.85)
    for ax, ylabel, title in [
        (axes2[0], "P_max Hertz [MPa]", "접촉 압력  (볼록일수록 높음)"),
        (axes2[1], "접촉 면적 [mm²]",   "접촉 면적  (편평일수록 큼)"),
    ]:
        ax.set_ylabel(ylabel); ax.set_xlabel("Time [s]")
        ax.set_title(title, fontsize=10); ax.grid(True, alpha=0.3)
    axes2[0].legend(fontsize=7, ncol=2)
    fig2.suptitle("압력·면적 시계열  (고정 τ 모드)", fontsize=11, fontweight='bold')
    fig2.tight_layout()
    figs.append(fig2)

    # ── Fig 3: 요약 메트릭  vs CV ─────────────────────────────────
    J_A     = np.array([r.J            for r in results_A])
    Fmax_A  = np.array([r.Fy_max       for r in results_A])
    Pmax_A  = np.array([r.P_max_MPa    for r in results_A])
    Amax_A  = np.array([r.A_max_mm2    for r in results_A])
    Tcon_A  = np.array([r.T_contact_s  for r in results_A])
    dmax_A  = np.array([r.delta_max_um for r in results_A])

    J_B     = np.array([r.J            for r in results_B])
    Fmax_B  = np.array([r.Fy_max       for r in results_B])
    Pmax_B  = np.array([r.P_max_MPa    for r in results_B])
    Amax_B  = np.array([r.A_max_mm2    for r in results_B])
    tau_B   = np.array([r.tau          for r in results_B])

    fig3, axes3 = plt.subplots(2, 3, figsize=(16, 9))
    axes3 = axes3.flatten()

    def _plot_pair(ax, x, yA, yB, ylabel, title, key=False):
        ax.plot(x, yA, 'o-', color='#1f77b4', lw=2, ms=7, label='A: 고정 τ')
        ax.plot(x, yB, 's--', color='#d62728', lw=2, ms=7, label='B: Hertz τ')
        ax.set_xlabel("CV (곡률)"); ax.set_ylabel(ylabel)
        ax.set_title(title, fontsize=10,
                     fontweight='bold' if key else 'normal')
        ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
        if key:
            ax.set_facecolor("#fff8e1")
            for yval, tag, col in [(yA, 'A', '#1f77b4'), (yB, 'B', '#d62728')]:
                ax.axhline(yval.mean(), color=col, ls=':', lw=1,
                           label=f'평균{tag}={yval.mean():.4f}')
            ax.legend(fontsize=7)
        # 보조 X축: Rs
        ax2 = ax.twiny()
        ax2.set_xlim(ax.get_xlim())
        ax2.set_xticks(x)
        ax2.set_xticklabels([f"{v:.0f}" for v in Rs_np],
                            fontsize=6, rotation=30)
        ax2.set_xlabel("Rs [mm]", fontsize=7)

    _plot_pair(axes3[0], CVS_np, J_A,    J_B,    "J [N·s]",   "★ 충격량 J",   key=True)
    _plot_pair(axes3[1], CVS_np, Fmax_A, Fmax_B, "F_max [N]", "최대 힘")
    _plot_pair(axes3[2], CVS_np, Pmax_A, Pmax_B, "P_max [MPa]","최대 압력")
    _plot_pair(axes3[3], CVS_np, Amax_A, Amax_B, "A_max [mm²]","최대 접촉 면적")
    _plot_pair(axes3[4], CVS_np, Tcon_A,
               np.array([r.T_contact_s for r in results_B]),
               "T_contact [s]", "접촉 지속 시간")

    # 마지막 칸: τ 보정 값 vs CV
    axes3[5].plot(CVS_np, np.array([r.tau for r in results_A]) * 1e3,
                  'o-', color='#1f77b4', lw=2, ms=7, label='A: 고정 τ [ms]')
    axes3[5].plot(CVS_np, tau_B * 1e3,
                  's--', color='#d62728', lw=2, ms=7, label='B: Hertz τ [ms]')
    axes3[5].set_xlabel("CV"); axes3[5].set_ylabel("τ [ms]")
    axes3[5].set_title("solref τ 비교", fontsize=10)
    axes3[5].legend(fontsize=8); axes3[5].grid(True, alpha=0.3)

    fig3.suptitle(
        f"요약 메트릭  R={R_MM}mm AR={AR_VAL}  |  "
        "★ 충격량이 CV와 무관한지 확인",
        fontsize=11, fontweight='bold')
    fig3.tight_layout()
    figs.append(fig3)

    # ── Fig 4: 점접촉 ↔ 면접촉 물리 해석  (Rs 기준) ─────────────────
    fig4, ax4 = plt.subplots(figsize=(10, 5))

    # Hertz 이론 예측선 (정규화)
    Rs_th  = np.linspace(Rs_np.min(), Rs_np.max(), 200)
    # P_max ∝ Rs^(-2/3)
    P_th   = Pmax_A[0] * (Rs_np[0] / Rs_th) ** (2.0 / 3.0) if Pmax_A[0] > 0 else Rs_th * 0
    # A_max ∝ Rs^(2/5)
    A_th   = Amax_A[0] * (Rs_th / Rs_np[0]) ** (2.0 / 5.0) if Amax_A[0] > 0 else Rs_th * 0

    ax_l = ax4
    ax_r = ax4.twinx()

    ax_l.scatter(Rs_np, Pmax_A, s=90, color='tab:red',   zorder=5,
                 label='P_max 시뮬 (고정τ) [MPa]')
    ax_l.plot(Rs_th,  P_th,  '--', color='tab:red',   alpha=0.4, lw=1.5,
              label='Hertz 이론: P∝Rs⁻²/³')
    ax_l.set_ylabel("P_max [MPa]", color='tab:red', fontsize=10)

    ax_r.scatter(Rs_np, Amax_A, s=90, color='tab:green', zorder=5, marker='s',
                 label='A_max 시뮬 (고정τ) [mm²]')
    ax_r.plot(Rs_th,  A_th,  '--', color='tab:green', alpha=0.4, lw=1.5,
              label='Hertz 이론: A∝Rs²/⁵')
    ax_r.set_ylabel("A_max [mm²]", color='tab:green', fontsize=10)

    ax_l.set_xlabel("Rs [mm]  —  작을수록 볼록(점접촉), 클수록 편평(면접촉)",
                    fontsize=10)
    ax4.set_title(
        "점접촉 ↔ 면접촉 전환\n"
        "Rs 작음 = 볼록(CV↑) → 높은 압력·작은 면적   |   "
        "Rs 큼 = 편평(CV↓) → 낮은 압력·큰 면적",
        fontsize=10, fontweight='bold')

    lines_l, labs_l = ax_l.get_legend_handles_labels()
    lines_r, labs_r = ax_r.get_legend_handles_labels()
    ax4.legend(lines_l + lines_r, labs_l + labs_r, fontsize=8, loc='center right')
    ax4.grid(True, alpha=0.3)
    fig4.tight_layout()
    figs.append(fig4)

    return figs


# ─── CSV 저장 ─────────────────────────────────────────────────────────────────
def save_csv(results_A: list[CVResult],
             results_B: list[CVResult],
             path: str) -> None:
    header = [
        "CV", "Rs_mm",
        "tau_A",    "J_A",    "Fmax_A",  "Pmax_A_MPa",  "Amax_A_mm2",  "Tcon_A_s",  "dmax_A_um",
        "tau_B",    "J_B",    "Fmax_B",  "Pmax_B_MPa",  "Amax_B_mm2",  "Tcon_B_s",  "dmax_B_um",
        "J_ratio_B_A",   # B/A 충격량 비율 (≈1이면 보존)
        "P_ratio_B_A",   # B/A 압력 비율
    ]
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["# CV Sweep Experiment —", datetime.now().isoformat()])
        w.writerow([f"# R_mm={R_MM}  AR={AR_VAL}  density={DENSITY}  "
                    f"E*={E_STAR/1e9:.1f}GPa  F0={HERTZ_F0_N}N"])
        w.writerow(header)
        for a, b in zip(results_A, results_B):
            J_ratio = b.J / a.J if a.J > 1e-9 else float('nan')
            P_ratio = b.P_max_MPa / a.P_max_MPa if a.P_max_MPa > 1e-9 else float('nan')
            w.writerow([
                f"{a.cv:.2f}", f"{a.Rs_mm:.2f}",
                f"{a.tau:.6f}", f"{a.J:.5f}", f"{a.Fy_max:.4f}",
                f"{a.P_max_MPa:.4f}", f"{a.A_max_mm2:.6f}",
                f"{a.T_contact_s:.4f}", f"{a.delta_max_um:.3f}",
                f"{b.tau:.6f}", f"{b.J:.5f}", f"{b.Fy_max:.4f}",
                f"{b.P_max_MPa:.4f}", f"{b.A_max_mm2:.6f}",
                f"{b.T_contact_s:.4f}", f"{b.delta_max_um:.3f}",
                f"{J_ratio:.4f}", f"{P_ratio:.4f}",
            ])


# ─── 메인 ─────────────────────────────────────────────────────────────────────
def main():
    os.makedirs(_RESULT_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    print("=" * 65)
    print("  CV 배치 스위프  —  점접촉 vs 면접촉 충격량 분석")
    print(f"  R={R_MM}mm  AR={AR_VAL}  density={DENSITY:.0f} kg/m³")
    print(f"  SIM_DURATION={SIM_DURATION}s  CVS={len(CVS)}개")
    print("=" * 65)

    # ─── 모드 A: 고정 τ ────────────────────────────────────────────
    tau_fixed = density_to_solref_tau(DENSITY)
    print(f"\n[모드 A] 고정 τ = {tau_fixed:.5f} s  (모든 CV 동일)\n")
    results_A: list[CVResult] = []
    for cv in CVS:
        r = run_one_cv(cv, tau=tau_fixed, mode="fixed")
        results_A.append(r)

    # ─── 모드 B: Hertz 보정 τ ──────────────────────────────────────
    print(f"\n[모드 B] Hertz 보정 τ  (E*={E_STAR/1e9:.1f} GPa, F₀={HERTZ_F0_N}N)\n")
    results_B: list[CVResult] = []
    for cv in CVS:
        Rs_m  = _rs_mm(R_MM, cv) * 1e-3
        tau_h = _tau_hertz(Rs_m)
        r = run_one_cv(cv, tau=tau_h, mode="hertz")
        results_B.append(r)

    # ─── 요약 테이블 출력 ──────────────────────────────────────────
    print(f"\n{'CV':>5} {'Rs':>6}  "
          f"{'J_A':>8} {'F_A':>7} {'P_A':>7} {'A_A':>7}  "
          f"{'τ_B(ms)':>8} {'J_B':>8} {'F_B':>7} {'P_B':>7}  "
          f"{'J_B/J_A':>8}")
    print("─" * 90)
    for a, b in zip(results_A, results_B):
        jr = b.J / a.J if a.J > 1e-9 else float('nan')
        print(f"{a.cv:>5.2f} {a.Rs_mm:>6.1f}  "
              f"{a.J:>8.4f} {a.Fy_max:>7.2f} {a.P_max_MPa:>7.3f} {a.A_max_mm2:>7.4f}  "
              f"{b.tau*1e3:>8.3f} {b.J:>8.4f} {b.Fy_max:>7.2f} {b.P_max_MPa:>7.3f}  "
              f"{jr:>8.4f}")

    print("\n★ J_B/J_A ≈ 1.0 → 충격량은 곡률과 무관 (이론 예측 일치)")
    print("★ P_max: CV ↑ → 볼록 → 점접촉 → 압력 집중 (CV 민감)")

    # ─── CSV ────────────────────────────────────────────────────────
    csv_path = os.path.join(_RESULT_DIR, f"cv_sweep_{ts}.csv")
    save_csv(results_A, results_B, csv_path)
    print(f"\n  ✔ CSV: {csv_path}")

    # ─── 개별 CV PNG 10개 ───────────────────────────────────────────
    print("\n[개별 CV 플롯] CV별 PNG 10개 저장 중...\n")
    plot_individual_cvs(results_A, results_B, _RESULT_DIR, ts)

    # ─── 종합 요약 플롯 4종 ─────────────────────────────────────────
    print("\n[종합 플롯] 요약 4종 저장 중...\n")
    figs = plot_sweep(results_A, results_B)
    tags = ["F_timeseries", "P_A_timeseries", "summary_metrics", "contact_type"]
    for fig, tag in zip(figs, tags):
        p = os.path.join(_RESULT_DIR, f"cv_sweep_{ts}_{tag}.png")
        fig.savefig(p, dpi=150, bbox_inches="tight")
        print(f"  ✔ {tag}.png")

    plt.show()


if __name__ == "__main__":
    main()

"""
crusher_tablet_sim.py  [v7 — 밀도 기반 접촉 경도 (solref 자동 계산)]
Crusher + Tablet 통합 시뮬레이션

▶ 알약 고정 방식: mocap body
    - freejoint 없음 → 관통 없음
    - data.mocap_pos / data.mocap_quat 으로 위치 제어
    - 충돌력은 data.contact + mj_contactForce 로 수집

▶ Phase 1  (PHASE1_STEPS 스텝, 뷰어 없음)
    lock_crank equality 활성 (-90°) + mocap 알약 배치 → 메커니즘 안정화

▶ Phase 2  (뷰어 오픈)
    MOTOR_DELAY 초 후 lock_crank 해제 → Motor CCW 구동 → 접촉력 기록
    ★ Moving-window stall 감지: 크랭크 속도가 STALL_VEL_THR 미만으로
      STALL_TIME_S 초 연속 유지 시 방향 전환 (CCW ↔ CW 반복)
      기본 2.0s → "지긋이 누르는 느낌"

▶ 배치 좌표 (MuJoCo world frame)
    PLACE_X_MM = -47.879  →  MuJoCo X
    PLACE_Z_MM =  50.108  →  MuJoCo Z
    WALL_Y_MM  = 336.199  →  MuJoCo Y  (충돌판 벽 표면)
    알약 중심 Y = WALL_Y_MM - half_th  (두께 절반만큼 앞에 → 표면이 벽에 접촉)

▶ 밀도 기반 접촉 경도 (--density / --mass)
    알약 밀도 [kg/m³] → MuJoCo solref 시정수 τ 자동 계산
    이론적 근거 (Hertzian Contact Theory):
      실제 정제: 압착 압력↑ → 밀도↑ → Young's modulus E↑
      E ∝ ρⁿ (n≈2~3)  →  K_contact ∝ 1/τ²  →  τ ∝ ρ^(-n/2)
    결과:
      밀도 높음 → τ 작음 → 강성 높음 → 관통 감소 → 경질 알약 모사
      밀도 낮음 → τ 큼   → 강성 낮음 → 관통 증가 → 연질 알약 모사

    사용법:
      --density 1400   밀도 직접 지정 [kg/m³]
      --mass    320    실측 질량(mg) → 형상 파라미터로 밀도 자동 계산

▶ 실행
    conda activate isaac_sim
    python crusher_tablet_sim.py [tablet.stl]
    python crusher_tablet_sim.py [tablet.stl] --density 1400
    python crusher_tablet_sim.py [tablet.stl] --mass 320
"""

import os
import sys
import re
import csv
import math
import argparse
import xml.etree.ElementTree as ET
from collections import deque
from datetime import datetime

import numpy as np
# NumPy 2.0 호환: trapz → trapezoid (try/except — getattr default는 먼저 평가됨)
try:
    _np_trapz = np.trapezoid
except AttributeError:
    _np_trapz = np.trapz
import matplotlib
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'Malgun Gothic'
plt.rcParams['axes.unicode_minus'] = False
import mujoco
import mujoco.viewer

# ── 경로 ─────────────────────────────────────────────────────────────
_HERE     = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.normpath(
    os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml"))
MJCF_DIR  = os.path.dirname(MJCF_PATH)
STL_DIR   = os.path.normpath(
    os.path.join(_HERE, "..", "..", "tablets_stl", "stl"))

# ── 결과 저장 디렉토리 ────────────────────────────────────────────────
_SIM_RESULT = os.path.normpath(os.path.join(_HERE, "..", "Sim_result"))
CSV_DIR     = os.path.join(_SIM_RESULT, "csv")
PLOT_DIR    = os.path.join(_SIM_RESULT, "plot")

# ── 배치 좌표 (mm, MuJoCo world frame) ──────────────────────────────
PLACE_X_MM = -47.879
PLACE_Z_MM =  50.108
WALL_Y_MM  = 336.199   # impact plate 벽 표면 Y [mm]
#   알약 중심 Y = WALL_Y_MM - half_th  (두께의 절반만큼 벽 앞에 → 표면이 벽에 정확히 접촉)
#   half_th = th / 2  where th = R_mm * 0.20 + 2 * (CV * 2 * R_mm)

# ── Phase 파라미터 ────────────────────────────────────────────────────
PHASE1_STEPS = 500
SIM_DURATION = 30.0
MOTOR_CTRL   = -0.5
MOTOR_DELAY  =  3.0

# ── Moving-window stall 감지 ─────────────────────────────────────────
STALL_TIME_S  = 2.0
STALL_VEL_THR = 0.05

# ── 실시간 플롯 갱신 주기 ─────────────────────────────────────────────
RT_PLOT_INTERVAL = 20

# ── 알약 초기 자세 (쿼터니언) ────────────────────────────────────────
_s = np.sqrt(2.0) / 2.0
TAB_QUAT = np.array([_s, 0.0, _s, 0.0])   # [qw, qx, qy, qz]

# ── 밀도 기반 접촉 경도 파라미터 ─────────────────────────────────────
#
#   물리 근거 (Hertzian Contact Theory):
#     실제 정제:  압착 압력↑ → 밀도↑ → Young's modulus E↑
#     E ∝ ρⁿ  (n ≈ 2~3,  경험적 관계)
#     MuJoCo: K_contact ∝ 1/τ²  →  τ ∝ ρ^(-n/2)
#     결과:   밀도 높을수록 τ 작게 = 접촉 강성 높게 = 관통 적게
#
#   기준점 (두 점으로 power-law 지수 α 자동 결정)
DENSITY_REF_SOFT  = 900.0    # kg/m³  연질 기준 (저압착 포도당 등)
DENSITY_REF_HARD  = 1800.0   # kg/m³  경질 기준 (탄산칼슘 등)
SOLREF_TAU_SOFT   = 0.020    # s      연질 → MuJoCo 기본 시정수 (가장 소프트)
SOLREF_TAU_HARD   = 0.002    # s      경질 → 실용적 최솟값 (더 작으면 불안정)
DENSITY_DEFAULT   = 1200.0   # kg/m³  기본값 (--density/--mass 미지정 시)
BICONVEX_VOL_FACTOR = 0.82   # biconvex 타원체 부피 보정계수 (타원체 대비 ≈18% 작음)


# ─────────────────────────────────────────────────────────────────────
def _parse_params(fname: str):
    """파일명에서 R, AR, CV 파라미터 추출."""
    stem = os.path.splitext(os.path.basename(fname))[0]
    m = re.search(r"R([\d.]+)_AR([\d.]+)_CV([\d.]+)", stem)
    if not m:
        return None, None, None
    return float(m.group(1)), float(m.group(2)), float(m.group(3))


# ─────────────────────────────────────────────────────────────────────
def estimate_tablet_volume_mm3(R_mm: float, AR: float, CV: float) -> float:
    """
    STL 파라미터 → biconvex 알약 부피 근사 [mm³].

    타원체 근사: V = (4/3)π × a × b × c × BICONVEX_VOL_FACTOR
      a = R_mm × AR  장반경 [mm]
      b = R_mm       단반경 [mm]
      c = half_th    반두께 [mm]
    BICONVEX_VOL_FACTOR = 0.82: biconvex는 외접 타원체보다 약 18% 작음
    """
    cd = CV * 2 * R_mm
    th = R_mm * 0.20 + 2 * cd
    a  = R_mm * AR
    b  = R_mm
    c  = th / 2.0
    return (4.0 / 3.0) * math.pi * a * b * c * BICONVEX_VOL_FACTOR


def mass_to_density(mass_mg: float, R_mm: float, AR: float, CV: float) -> float:
    """
    실측 질량(mg) + 형상 파라미터 → 밀도 [kg/m³].
    부피는 estimate_tablet_volume_mm3() 로 추정.
    """
    mass_kg = mass_mg * 1e-6
    vol_m3  = estimate_tablet_volume_mm3(R_mm, AR, CV) * 1e-9  # mm³ → m³
    return mass_kg / vol_m3


def density_to_solref_tau(density_kg_m3: float) -> float:
    """
    밀도 [kg/m³] → MuJoCo solref 시정수 τ [s].

    Power-law 보간:
        τ(ρ) = τ_soft × (ρ_soft / ρ)^α
        α = log(τ_hard/τ_soft) / log(ρ_hard/ρ_soft)  ≈ −3.32

    ρ 범위는 [DENSITY_REF_SOFT, DENSITY_REF_HARD] 로 클램핑.
    τ 범위는 [SOLREF_TAU_HARD,   SOLREF_TAU_SOFT]  로 클램핑.

    밀도 예시:
        900  → τ=0.0200 s  (연질, 기본 MuJoCo)
       1200  → τ=0.0080 s  (중간)
       1600  → τ=0.0033 s  (경질)
       1800  → τ=0.0020 s  (초경질)
    """
    rho   = float(np.clip(density_kg_m3, DENSITY_REF_SOFT, DENSITY_REF_HARD))
    alpha = math.log(SOLREF_TAU_HARD / SOLREF_TAU_SOFT) / \
            math.log(DENSITY_REF_HARD / DENSITY_REF_SOFT)
    tau   = SOLREF_TAU_SOFT * (DENSITY_REF_SOFT / rho) ** alpha
    return float(np.clip(tau, SOLREF_TAU_HARD, SOLREF_TAU_SOFT))


# ─────────────────────────────────────────────────────────────────────
def _build_model(stl_path: str, R_mm: float, half_th: float,
                 density_kg_m3: float = DENSITY_DEFAULT):
    """
    Crusher XML + Tablet STL → MjModel (메모리 내 조합).

      ① meshdir  → Crusher MJCF 디렉토리 절대경로
      ② keyframe 제거  → nq 불일치 방지
      ③ tablet body    → mocap="true" + geom (freejoint/site/sensor 없음)

    배치 기준:
      알약 중심 Y = WALL_Y_MM - half_th
        → 두께의 절반만큼 벽 앞에 배치하여 표면이 벽면에 딱 닿음

    접촉 경도:
      density_kg_m3 → density_to_solref_tau() → solref τ
      밀도 높음 → τ 작음 → 강성 높음 → 관통 감소 (경질 알약 모사)
    """
    pos_x = PLACE_X_MM * 1e-3
    pos_z = PLACE_Z_MM * 1e-3
    pos_y = (WALL_Y_MM - half_th) * 1e-3

    # ── 밀도 → solref/solimp 계산 ────────────────────────────────────
    tau      = density_to_solref_tau(density_kg_m3)
    dimp_max = float(np.interp(density_kg_m3,
                               [DENSITY_REF_SOFT, DENSITY_REF_HARD],
                               [0.950, 0.999]))
    solref_str = f"{tau:.6f} 1"
    solimp_str = f"0.99 {dimp_max:.4f} 0.0001"

    center_y_mm = WALL_Y_MM - half_th
    print(f"  배치 [mm] : X={PLACE_X_MM:.3f}  "
          f"Y_wall={WALL_Y_MM:.3f}  Y_center={center_y_mm:.3f}  "
          f"Z={PLACE_Z_MM:.3f}  (offset=-th/2={-half_th:.3f}mm)")
    print(f"  배치 [m]  : X={pos_x:.5f}  Y={pos_y:.5f}  Z={pos_z:.5f}")
    print(f"  접촉 경도 : density={density_kg_m3:.0f} kg/m³  "
          f"→ solref τ={tau:.5f}s  solimp_dmax={dimp_max:.4f}")

    # ── Crusher XML 파싱 ─────────────────────────────────────────────
    tree = ET.parse(MJCF_PATH)
    root = tree.getroot()

    # ① meshdir
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", MJCF_DIR)

    # ② keyframe 제거
    for kf in root.findall("keyframe"):
        root.remove(kf)

    # ③-a tablet mesh + material
    asset = root.find("asset")
    ET.SubElement(asset, "mesh", {
        "name":  "tablet_mesh",
        "file":  "tablet.stl",
        "scale": ".001 .001 .001",
    })
    ET.SubElement(asset, "material", {
        "name":      "tablet_mat",
        "rgba":      ".85 .80 .72 1",
        "specular":  ".4",
        "shininess": ".3",
    })

    # ③-b tablet body — mocap="true"
    worldbody = root.find("worldbody")
    tab = ET.SubElement(worldbody, "body", {
        "name":  "tablet",
        "mocap": "true",
        "pos":   f"{pos_x:.6f} {pos_y:.6f} {pos_z:.6f}",
        "quat":  " ".join(f"{v:.7f}" for v in TAB_QUAT),
    })
    ET.SubElement(tab, "geom", {
        "name":     "tablet_geom",
        "type":     "mesh",
        "mesh":     "tablet_mesh",
        "material": "tablet_mat",
        "density":  f"{density_kg_m3:.1f}",   # 실제 밀도로 질량/관성 계산
        "condim":   "4",
        "friction": ".5 .02 .01",
        "solref":   solref_str,                # 밀도 기반 접촉 강성
        "solimp":   solimp_str,                # 밀도 기반 임피던스
    })

    # ── 조합 → MjModel ───────────────────────────────────────────────
    xml_str   = ET.tostring(root, encoding="unicode")
    stl_bytes = open(stl_path, "rb").read()
    model     = mujoco.MjModel.from_xml_string(
        xml_str, assets={"tablet.stl": stl_bytes})
    return model, (pos_x, pos_y, pos_z)


# ─────────────────────────────────────────────────────────────────────
def _sum_contact_force(model, data, body_id) -> np.ndarray:
    """
    body_id에 작용하는 모든 접촉력의 합 (world frame, XYZ) [N].
    mj_contactForce → contact frame → world frame 변환.
    """
    f_total = np.zeros(3)
    force6  = np.zeros(6)
    for i in range(data.ncon):
        c     = data.contact[i]
        g1_b  = model.geom_bodyid[c.geom1]
        g2_b  = model.geom_bodyid[c.geom2]
        if body_id not in (g1_b, g2_b):
            continue
        mujoco.mj_contactForce(model, data, i, force6)
        frame   = c.frame.reshape(3, 3)
        f_world = frame.T @ force6[:3]
        if g2_b == body_id:
            f_world = -f_world
        f_total += f_world
    return f_total


# ─────────────────────────────────────────────────────────────────────
def run(stl_path: str, density_kg_m3: float = DENSITY_DEFAULT):
    print("=" * 62)
    print("  Crusher + Tablet  2-Phase 통합 시뮬레이션  [mocap 알약]")
    print("=" * 62)

    fname = os.path.basename(stl_path)
    R_mm, AR, CV = _parse_params(fname)
    if R_mm is None:
        print(f"[ERROR] 파일명 파싱 실패: {fname}")
        sys.exit(1)

    cd      = CV * 2 * R_mm
    th      = R_mm * 0.20 + 2 * cd
    half_th = th / 2.0

    # ── 밀도 → 접촉 경도 사전 계산 (CSV 기록용) ────────────────────
    vol_mm3  = estimate_tablet_volume_mm3(R_mm, AR, CV)
    tau      = density_to_solref_tau(density_kg_m3)
    dimp_max = float(np.interp(density_kg_m3,
                               [DENSITY_REF_SOFT, DENSITY_REF_HARD],
                               [0.950, 0.999]))

    print(f"  STL  : {fname}")
    print(f"  R={R_mm:.1f}mm  AR={AR:.2f}  CV={CV:.2f}  "
          f"두께≈{th:.2f}mm  half_th={half_th:.2f}mm")
    print(f"  밀도={density_kg_m3:.0f} kg/m³  "
          f"부피≈{vol_mm3:.1f} mm³  "
          f"→ τ={tau:.5f}s  solimp_dmax={dimp_max:.4f}")
    print()

    # ── 결과 저장 경로 설정 ─────────────────────────────────────────
    os.makedirs(CSV_DIR,  exist_ok=True)
    os.makedirs(PLOT_DIR, exist_ok=True)
    stem        = os.path.splitext(fname)[0]
    ts          = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_stem = f"{stem}__{ts}"
    print(f"  결과 저장: {_SIM_RESULT}")
    print(f"  파일 prefix: {result_stem}\n")

    # ── 모델 로드 ────────────────────────────────────────────────────
    model, (px, py, pz) = _build_model(stl_path, R_mm, half_th, density_kg_m3)
    data = mujoco.MjData(model)

    # ── ID 조회 ──────────────────────────────────────────────────────
    crank_jid  = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT,    "L3_Bevel_GearBox_1_L4_Shaft_1")
    act_crank  = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Motor1_crank")
    b_tablet   = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY,     "tablet")
    b_slider   = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY,     "L8_Link3_Shaft_1")
    eq_lock_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_EQUALITY, "lock_crank")

    crank_qadr = model.jnt_qposadr[crank_jid]
    crank_vadr = model.jnt_dofadr[crank_jid]
    mocap_id   = model.body_mocapid[b_tablet]

    print(f"\n  nq={model.nq}  |  crank qpos[{crank_qadr}] qvel[{crank_vadr}]  "
          f"mocap_id={mocap_id}  lock_crank eq_id={eq_lock_id}")

    # ── ❶ 초기 상태 ──────────────────────────────────────────────────
    data.qpos[crank_qadr]     = -np.pi / 2
    data.qvel[:]              = 0.0
    data.mocap_pos[mocap_id]  = [px, py, pz]
    data.mocap_quat[mocap_id] = TAB_QUAT
    mujoco.mj_forward(model, data)

    # ── ❷ Phase 1: 안정화 ────────────────────────────────────────────
    print(f"\n◆ Phase 1: {PHASE1_STEPS} 스텝 안정화 (뷰어 없음)")
    for _ in range(PHASE1_STEPS):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    print(f"  ✔ Phase 1 완료  crank={np.degrees(data.qpos[crank_qadr]):.1f}°"
          f"  sim_time={data.time:.3f}s")

    # ── ❸ Phase 2 준비 ───────────────────────────────────────────────
    _dt          = float(model.opt.timestep)
    stall_window = max(1, int(round(STALL_TIME_S / _dt)))

    print(f"\n◆ Phase 2: 뷰어 오픈  (lock_crank 활성)")
    print(f"  {MOTOR_DELAY:.1f}s 후 lock_crank 해제 → {MOTOR_CTRL} N·m CCW")
    print(f"  stall: |ω|<{STALL_VEL_THR} rad/s × {stall_window} 스텝"
          f" ({STALL_TIME_S:.1f}s) → 방향 전환")
    print(f"  측정 시간 = {SIM_DURATION} s\n")

    data.ctrl[act_crank] = 0.0
    phase2_start_t = data.time

    motor_on   = False
    motor_on_t = None
    motor_dir  = 0
    stall_buf  = deque(maxlen=stall_window)
    rev_events = []

    t_log    = []
    f_log    = []
    vel_log  = []
    dir_log  = []
    ncon_log = []
    slider_y = []
    tablet_y = []
    gap_log  = []
    first_contact_t = None

    print(f"  {'Time':>6s} | {'Slider_Y':>9s} mm | {'Tablet_Y':>9s} mm | "
          f"{'Gap':>7s} mm | {'F_Y(N)':>8s} | {'ω(rad/s)':>9s} | {'ncon':>4s}")
    print("  " + "-" * 82)

    # ── 실시간 반력 플롯 초기화 ──────────────────────────────────────
    plt.ion()
    fig_rt, ax_rt = plt.subplots(figsize=(9, 3))
    fig_rt.suptitle(
        f"실시간 법선 반력 F_Y [N]  |  ρ={density_kg_m3:.0f} kg/m³  τ={tau:.4f}s",
        fontsize=10)
    line_fy, = ax_rt.plot([], [], color="tab:blue", lw=1.5, label="F_Y [N]")
    ax_rt.axhline(0, color="k", lw=0.5, ls="--")
    ax_rt.set_xlabel("Time [s]")
    ax_rt.set_ylabel("F_Y [N]")
    ax_rt.legend(fontsize=9)
    ax_rt.grid(True, alpha=0.3)
    fig_rt.tight_layout()
    fig_rt.canvas.draw()
    fig_rt.canvas.flush_events()
    _rt_vlines: list = []

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT]      = False
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = False
        viewer.opt.frame                                      = mujoco.mjtFrame.mjFRAME_NONE
        viewer.opt.geomgroup[3]                               = False
        for sg in range(5):
            viewer.opt.sitegroup[sg] = False

        while viewer.is_running() and data.time < SIM_DURATION:

            # 모터 ON
            if not motor_on and (data.time - phase2_start_t) >= MOTOR_DELAY:
                data.eq_active[eq_lock_id] = 0
                motor_dir = 1
                data.ctrl[act_crank] = motor_dir * MOTOR_CTRL
                motor_on   = True
                motor_on_t = data.time
                stall_buf.clear()
                print(f"  *** lock 해제 + 모터 ON: t={data.time:.3f}s "
                      f"ctrl={motor_dir * MOTOR_CTRL:.2f} N·m (CCW) ***")

            # Moving-window stall 감지
            if motor_on:
                crank_vel = abs(data.qvel[crank_vadr])
                stall_buf.append(crank_vel < STALL_VEL_THR)
                if len(stall_buf) == stall_window and all(stall_buf):
                    motor_dir = -motor_dir
                    data.ctrl[act_crank] = motor_dir * MOTOR_CTRL
                    stall_buf.clear()
                    dir_str = "CCW" if motor_dir > 0 else "CW"
                    rev_events.append((data.time, dir_str))
                    print(f"  *** 방향 전환 → {dir_str}  t={data.time:.3f}s "
                          f"ctrl={motor_dir * MOTOR_CTRL:.2f} N·m  "
                          f"ω={crank_vel:.4f} rad/s ***")

            mujoco.mj_step(model, data)

            sy     = float(data.xpos[b_slider, 1])
            ty_now = float(data.xpos[b_tablet, 1])
            gap_mm = (ty_now - sy) * 1e3
            omega  = float(data.qvel[crank_vadr])
            fc_now = _sum_contact_force(model, data, b_tablet)

            t_log.append(data.time)
            f_log.append(fc_now.copy())
            vel_log.append(omega)
            dir_log.append(motor_dir)
            ncon_log.append(data.ncon)
            slider_y.append(sy)
            tablet_y.append(ty_now)
            gap_log.append(gap_mm)

            if first_contact_t is None and data.ncon > 0:
                for ci in range(data.ncon):
                    c = data.contact[ci]
                    if b_tablet in (model.geom_bodyid[c.geom1],
                                    model.geom_bodyid[c.geom2]):
                        first_contact_t = data.time
                        print(f"  *** 첫 접촉: t={data.time:.3f}s  "
                              f"F_Y={fc_now[1]:.2f}N  gap={gap_mm:.2f}mm ***")
                        break

            if len(t_log) % 500 == 0:
                dir_lbl = {1: "CCW", -1: "CW", 0: "---"}.get(motor_dir, "?")
                print(f"  {data.time:6.2f}s | {sy*1e3:9.2f}    | "
                      f"{ty_now*1e3:9.2f}    | {gap_mm:7.2f}    | "
                      f"{fc_now[1]:8.3f} | {omega:9.4f} | {data.ncon:4d}"
                      f"  [{dir_lbl}]")

            # 실시간 플롯 갱신
            if len(t_log) % RT_PLOT_INTERVAL == 0 and len(t_log) > 1:
                fy_data = [f[1] for f in f_log]
                line_fy.set_data(t_log, fy_data)
                ax_rt.relim()
                ax_rt.autoscale_view()
                for vl in _rt_vlines:
                    try:
                        vl.remove()
                    except Exception:
                        pass
                _rt_vlines.clear()
                if motor_on_t is not None:
                    _rt_vlines.append(
                        ax_rt.axvline(motor_on_t, color="tab:orange",
                                      ls="--", lw=1.0, label="모터 ON"))
                for ev_t, ev_dir in rev_events:
                    _rt_vlines.append(
                        ax_rt.axvline(ev_t, color="tab:red",
                                      ls=":", lw=1.0, label=f"→{ev_dir}"))
                fig_rt.canvas.draw_idle()
                fig_rt.canvas.flush_events()

            viewer.sync()

    plt.ioff()

    # ── 결과 집계 ─────────────────────────────────────────────────────
    if not t_log:
        print("[경고] 데이터 없음.")
        return

    t   = np.array(t_log)
    fc  = np.array(f_log)
    vel = np.array(vel_log)
    sy  = np.array(slider_y) * 1e3
    ty  = np.array(tablet_y) * 1e3
    gap = np.array(gap_log)
    fc_mag  = np.linalg.norm(fc, axis=1)

    J_Y     = float(_np_trapz(fc[:, 1], t))
    F_Y_max = float(fc[:, 1].max())
    F_Y_min = float(fc[:, 1].min())

    print(f"\n  {'='*62}")
    print(f"  수집    : {len(t)} steps  ({t[-1]:.2f} s)")
    print(f"  Slider Y range  : {sy.min():.1f} ~ {sy.max():.1f} mm")
    print(f"  Tablet Y range  : {ty.min():.1f} ~ {ty.max():.1f} mm")
    print(f"  Min gap         : {gap.min():.2f} mm  (<0 = penetration)")
    print(f"  F_Y range       : {F_Y_min:.3f} ~ {F_Y_max:.3f} N")
    print(f"  |F| max         : {fc_mag.max():.3f} N")
    print(f"  Impulse J_Y     : {J_Y:.5f} N·s")
    print(f"  ω_crank range   : {vel.min():.4f} ~ {vel.max():.4f} rad/s")
    print(f"  방향 전환 횟수  : {len(rev_events)} 회")
    for i, (ev_t, ev_dir) in enumerate(rev_events, 1):
        print(f"    [{i}] t={ev_t:.3f}s → {ev_dir}")
    if first_contact_t:
        print(f"  First contact   : t = {first_contact_t:.3f} s")
    else:
        print("  [!] No contact detected")
    print(f"  {'='*62}")

    # ── 플롯 ─────────────────────────────────────────────────────────
    title_base = (f"Motor={MOTOR_CTRL} N·m  |  "
                  f"R={R_mm:.1f}mm AR={AR:.2f} CV={CV:.2f}  |  "
                  f"ρ={density_kg_m3:.0f} kg/m³ τ={tau:.4f}s")

    def _add_event_vlines(ax_):
        if motor_on_t is not None:
            ax_.axvline(motor_on_t, color="tab:orange", ls="--", lw=1.2,
                        label=f"모터 ON (t={motor_on_t:.2f}s)")
        for ev_t, ev_dir in rev_events:
            ax_.axvline(ev_t, color="tab:red", ls=":", lw=1.0,
                        label=f"→{ev_dir} (t={ev_t:.2f}s)")

    # 그림 1: 위치
    fig1, axes1 = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    fig1.suptitle(f"Position — {title_base}", fontsize=10, fontweight="bold")
    axes1[0].plot(t, sy, color="tab:orange", lw=1.5, label="Slider Y (L8)")
    axes1[0].plot(t, ty, color="tab:blue",   lw=1.5, label="Tablet Y (mocap)")
    axes1[0].axhline(WALL_Y_MM, color="tab:red", ls=":", lw=1.2,
                     label=f"Wall Y={WALL_Y_MM:.1f}mm")
    _add_event_vlines(axes1[0])
    axes1[0].set_ylabel("World Y [mm]"); axes1[0].legend(fontsize=8)
    axes1[0].grid(True, alpha=0.3)
    axes1[1].plot(t, gap, color="tab:purple", lw=1.5)
    axes1[1].axhline(0, color="tab:red", ls="--", lw=0.8, label="Contact (gap=0)")
    _add_event_vlines(axes1[1])
    axes1[1].set_ylabel("Gap [mm]"); axes1[1].set_xlabel("Time [s]")
    axes1[1].legend(fontsize=8); axes1[1].grid(True, alpha=0.3)
    fig1.tight_layout()

    # 그림 2: 법선 반력 + 임펄스
    fig2, axes2 = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    fig2.suptitle(f"Normal Contact Force (World-Y) — {title_base}",
                  fontsize=10, fontweight="bold")
    axes2[0].plot(t, fc[:, 1], color="tab:blue", lw=1.5,
                  label="F_Y  (tablet ← impact plate)")
    axes2[0].fill_between(t, 0, fc[:, 1], where=(fc[:, 1] > 0),
                           alpha=0.12, color="tab:blue", label="압축")
    axes2[0].fill_between(t, 0, fc[:, 1], where=(fc[:, 1] < 0),
                           alpha=0.12, color="tab:red",  label="인장")
    _add_event_vlines(axes2[0])
    axes2[0].set_ylabel("F_Y [N]")
    axes2[0].set_title(f"max={F_Y_max:.3f} N  min={F_Y_min:.3f} N")
    axes2[0].legend(fontsize=8); axes2[0].grid(True, alpha=0.3)
    J_cumul = np.cumsum(fc[:, 1]) * float(model.opt.timestep)
    axes2[1].plot(t, J_cumul, color="tab:green", lw=1.5,
                  label=f"누적 임펄스 J_Y = {J_Y:.4f} N·s")
    _add_event_vlines(axes2[1])
    axes2[1].set_ylabel("J_Y [N·s]"); axes2[1].set_xlabel("Time [s]")
    axes2[1].legend(fontsize=8); axes2[1].grid(True, alpha=0.3)
    fig2.tight_layout()

    # 그림 3: XYZ 성분
    fig3, axes3 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig3.suptitle("Contact Force Components (world frame)",
                  fontsize=10, fontweight="bold")
    for i, (lbl, col) in enumerate(
            [("X (lateral)", "tab:red"),
             ("Y (normal)",  "tab:blue"),
             ("Z (vertical)","tab:green")]):
        axes3[i].plot(t, fc[:, i], color=col, lw=1.2, label=f"F_{lbl}")
        axes3[i].axhline(0, color="k", lw=0.5)
        _add_event_vlines(axes3[i])
        axes3[i].set_ylabel("F [N]"); axes3[i].legend(fontsize=8)
        axes3[i].grid(True, alpha=0.3)
    axes3[2].set_xlabel("Time [s]")
    fig3.tight_layout()

    # 그림 4: 크랭크 각속도
    fig4, ax4 = plt.subplots(figsize=(11, 3))
    fig4.suptitle(f"Crank Angular Velocity — {title_base}",
                  fontsize=10, fontweight="bold")
    ax4.plot(t, vel, color="tab:purple", lw=1.2, label="ω crank [rad/s]")
    ax4.axhline(0, color="k", lw=0.5)
    ax4.axhline( STALL_VEL_THR, color="gray", ls="--", lw=0.8,
                 label=f"stall thr ±{STALL_VEL_THR}")
    ax4.axhline(-STALL_VEL_THR, color="gray", ls="--", lw=0.8)
    _add_event_vlines(ax4)
    ax4.set_ylabel("ω [rad/s]"); ax4.set_xlabel("Time [s]")
    ax4.legend(fontsize=8); ax4.grid(True, alpha=0.3)
    fig4.tight_layout()

    # ── CSV 저장 ──────────────────────────────────────────────────────
    _DIR_STR = {1: "CCW", -1: "CW", 0: "off"}
    csv_path = os.path.join(CSV_DIR, f"{result_stem}.csv")

    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["# Crusher Tablet Simulation — Force Profile"])
        w.writerow(["# Generated", datetime.now().isoformat(timespec="seconds")])
        w.writerow(["# STL file", fname])
        w.writerow(["# R_mm", R_mm, "AR", AR, "CV", CV,
                    "thickness_mm", f"{th:.3f}"])
        w.writerow(["# timestep_s", float(model.opt.timestep),
                    "solver", "Newton",
                    "iterations", int(model.opt.iterations)])
        w.writerow(["# MOTOR_CTRL_Nm", MOTOR_CTRL,
                    "MOTOR_DELAY_s", MOTOR_DELAY,
                    "SIM_DURATION_s", SIM_DURATION])
        w.writerow(["# STALL_TIME_S", STALL_TIME_S,
                    "STALL_WINDOW_steps", stall_window,
                    "STALL_VEL_THR_rad_s", STALL_VEL_THR])
        # ── 밀도 / 접촉 경도 메타데이터 ──────────────────────────────
        w.writerow(["# density_kg_m3", density_kg_m3,
                    "vol_estimate_mm3", f"{vol_mm3:.2f}",
                    "biconvex_factor", BICONVEX_VOL_FACTOR])
        w.writerow(["# solref_tau_s", f"{tau:.6f}",
                    "solimp_dmax", f"{dimp_max:.4f}",
                    "DENSITY_REF_SOFT", DENSITY_REF_SOFT,
                    "DENSITY_REF_HARD", DENSITY_REF_HARD])
        # ─────────────────────────────────────────────────────────────
        w.writerow(["# PLACE_X_mm", PLACE_X_MM,
                    "PLACE_Y_mm (wall)", WALL_Y_MM,
                    "PLACE_Z_mm", PLACE_Z_MM])
        w.writerow(["# F_Y_max_N", f"{F_Y_max:.5f}",
                    "F_Y_min_N", f"{F_Y_min:.5f}",
                    "Impulse_J_Y_Ns", f"{J_Y:.6f}"])
        w.writerow(["# direction_changes", len(rev_events)])
        for i, (ev_t, ev_dir) in enumerate(rev_events, 1):
            w.writerow([f"#   rev[{i}]", f"t={ev_t:.4f}s", f"dir={ev_dir}"])
        if first_contact_t:
            w.writerow(["# first_contact_s", f"{first_contact_t:.5f}"])
        w.writerow([])
        w.writerow(["Time_s",
                    "F_X_N", "F_Y_N", "F_Z_N", "F_mag_N",
                    "Slider_Y_mm", "Tablet_Y_mm", "Gap_mm",
                    "Crank_vel_rad_s", "Motor_dir", "ncon"])
        for i in range(len(t_log)):
            w.writerow([
                f"{t_log[i]:.5f}",
                f"{f_log[i][0]:.6f}",
                f"{f_log[i][1]:.6f}",
                f"{f_log[i][2]:.6f}",
                f"{fc_mag[i]:.6f}",
                f"{slider_y[i]*1e3:.4f}",
                f"{tablet_y[i]*1e3:.4f}",
                f"{gap_log[i]:.4f}",
                f"{vel_log[i]:.6f}",
                _DIR_STR.get(dir_log[i], "?"),
                ncon_log[i],
            ])
    print(f"\n  ✔ CSV 저장: {csv_path}")

    # ── 플롯 저장 ─────────────────────────────────────────────────────
    plot_specs = [
        (fig_rt, "realtime_force"),
        (fig1,   "position"),
        (fig2,   "force_magnitude"),
        (fig3,   "force_components"),
        (fig4,   "crank_velocity"),
    ]
    print(f"  ✔ Plot 저장 디렉토리: {PLOT_DIR}")
    for fig, tag in plot_specs:
        fname_png = f"{result_stem}__{tag}.png"
        p = os.path.join(PLOT_DIR, fname_png)
        fig.savefig(p, dpi=150, bbox_inches="tight")
        print(f"     {fname_png}")
    plt.show()


# ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Crusher + Tablet 2-Phase 통합 시뮬레이션 (mocap 알약)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "예시:\n"
            "  python crusher_tablet_sim.py tablet_R6.0_AR1.50_CV0.20.stl\n"
            "  python crusher_tablet_sim.py tablet.stl --density 1400\n"
            "  python crusher_tablet_sim.py tablet.stl --mass 320\n"
        ),
    )
    parser.add_argument("stl", nargs="?", default=None,
                        help="Tablet STL 파일 경로")

    # ── 밀도 / 질량 (상호 배타적) ────────────────────────────────────
    density_grp = parser.add_mutually_exclusive_group()
    density_grp.add_argument(
        "--density", type=float, default=None, metavar="KG_M3",
        help=f"알약 밀도 [kg/m³]  (기본: {DENSITY_DEFAULT:.0f})")
    density_grp.add_argument(
        "--mass", type=float, default=None, metavar="MG",
        help="알약 실측 질량 [mg] → 형상 파라미터로 밀도 자동 계산")

    # ── GUI 호환 옵션 (항상 저장하므로 플래그는 무시됨) ──────────────
    parser.add_argument(
        "--save-plots", action="store_true",
        help="플롯 PNG 저장  (기본: 항상 저장, GUI 호환용 플래그)")

    args = parser.parse_args()
    stl_path = args.stl

    if stl_path is None:
        try:
            import tkinter as tk
            from tkinter import filedialog
            root_tk = tk.Tk(); root_tk.withdraw()
            stl_path = filedialog.askopenfilename(
                title="Tablet STL 선택",
                initialdir=STL_DIR if os.path.isdir(STL_DIR) else "~",
                filetypes=[("STL Files", "*.stl")],
            )
            root_tk.destroy()
        except Exception:
            pass

    if not stl_path:
        print("사용법: python crusher_tablet_sim.py <path>.stl [--density KG_M3 | --mass MG]")
        sys.exit(0)

    stl_path = os.path.abspath(stl_path)
    if not os.path.exists(stl_path):
        print(f"[오류] 파일 없음: {stl_path}")
        sys.exit(1)

    # ── 밀도 결정 ─────────────────────────────────────────────────────
    density_kg_m3 = DENSITY_DEFAULT
    if args.density is not None:
        density_kg_m3 = float(args.density)
        print(f"  밀도 (직접 지정): {density_kg_m3:.1f} kg/m³")
    elif args.mass is not None:
        R_tmp, AR_tmp, CV_tmp = _parse_params(stl_path)
        if R_tmp is not None:
            density_kg_m3 = mass_to_density(args.mass, R_tmp, AR_tmp, CV_tmp)
            print(f"  질량 {args.mass:.1f} mg + 형상 파라미터 "
                  f"→ 밀도 {density_kg_m3:.1f} kg/m³")
        else:
            print("[경고] 파일명 파싱 실패 → 기본 밀도 사용 "
                  f"({DENSITY_DEFAULT:.0f} kg/m³)")

    run(stl_path, density_kg_m3=density_kg_m3)

"""
test_tdp_position.py  — 상사점(TDP) 위치 가설 검증
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
가설:
    슬라이더가 알약에 접촉하는 크랭크 각도가 상사점(dead center)에
    가까울수록 기계적 힘 증폭(Torque / Moment_Arm)이 커져 F_max가 증가한다.

    크랭크-슬라이더 기구에서:
        슬라이더 힘 = 모터 토크 / (유효 모멘트 암)
        유효 모멘트 암 = r * sin(θ)   (θ = 크랭크 각도)
        → 상사점(θ → 0°) 근처: 모멘트 암 → 0  → 힘 → ∞

방법:
    같은 알약 STL, mocap 고정 (알약은 움직이지 않음)
    알약 위치를 슬라이더 방향으로 y_offset mm씩 당기면서 실험
    → 슬라이더가 알약에 접촉하는 크랭크 각도 θ_contact 가 달라짐
    → F_max vs θ_contact 관계를 확인

기대 결과 (가설이 맞으면):
    y_offset 증가  →  θ_contact 상사점에서 멀어짐  →  F_max 감소
    → F_max vs θ_contact 그래프에서 단조 감소 관계

Usage:
    python test_tdp_position.py
    python test_tdp_position.py --stl tablet_R4.0_AR1.00_CV0.20.stl
    python test_tdp_position.py --offsets 0 5 10 20 30
    python test_tdp_position.py --R 4.0 --AR 1.00 --CV 0.20
"""

import os, sys, re, math, argparse, glob, xml.etree.ElementTree as ET
from datetime import datetime

import numpy as np
try:
    _np_trapz = np.trapezoid
except AttributeError:
    _np_trapz = np.trapz

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'Malgun Gothic'
plt.rcParams['axes.unicode_minus'] = False

# ── 경로 ─────────────────────────────────────────────────────────────
_HERE       = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH   = os.path.normpath(os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml"))
MJCF_DIR    = os.path.dirname(MJCF_PATH)
STL_DIR     = os.path.normpath(os.path.join(_HERE, "..", "..", "tablets_stl", "stl"))
_SIM_RESULT = os.path.normpath(os.path.join(_HERE, "..", "Sim_result"))
RESULT_DIR  = os.path.join(_SIM_RESULT, "tdp_test")

# ── 배치 좌표 ─────────────────────────────────────────────────────────
PLACE_X_MM = -47.879
PLACE_Z_MM =  50.108
WALL_Y_MM  = 336.199   # 충돌판 벽 표면 Y [mm]

# ── 시뮬 파라미터 ─────────────────────────────────────────────────────
PHASE1_STEPS = 500
SIM_DURATION = 15.0     # 한 스트로크 측정에 충분한 시간
MOTOR_CTRL   = -0.5     # [N·m] 토크 제어
MOTOR_DELAY  =  2.0     # [s] 모터 시작 지연

# ── 알약 접촉 파라미터 ────────────────────────────────────────────────
DENSITY          = 1200.0
DENSITY_REF_SOFT = 900.0
DENSITY_REF_HARD = 1800.0
SOLREF_TAU_SOFT  = 0.020
SOLREF_TAU_HARD  = 0.001

_s       = math.sqrt(2.0) / 2.0
_TAB_QUAT = [_s, 0.0, _s, 0.0]

# ── 테스트할 Y 오프셋 목록 [mm] ──────────────────────────────────────
# 0 = 알약 뒷면이 벽에 접함 (기준)
# X > 0 = 알약을 슬라이더 방향(-Y)으로 X mm 당김
DEFAULT_OFFSETS = [0, 3, 6, 9, 12, 15, 20, 25, 30]


# ─────────────────────────────────────────────────────────────────────
def _parse_params(fname):
    m = re.search(r"R([\d.]+)_AR([\d.]+)_CV([\d.]+)",
                  os.path.splitext(os.path.basename(fname))[0])
    return (float(m.group(1)), float(m.group(2)), float(m.group(3))) if m else (None, None, None)


def _density_tau(rho):
    rho   = float(np.clip(rho, DENSITY_REF_SOFT, DENSITY_REF_HARD))
    alpha = math.log(SOLREF_TAU_HARD / SOLREF_TAU_SOFT) / \
            math.log(DENSITY_REF_HARD / DENSITY_REF_SOFT)
    return float(np.clip(SOLREF_TAU_SOFT * (DENSITY_REF_SOFT / rho) ** alpha,
                         SOLREF_TAU_HARD, SOLREF_TAU_SOFT))


def _build_model(stl_path, half_th, y_offset_mm, density=DENSITY):
    """
    알약을 mocap(공간 고정)으로 배치.
    y_offset_mm > 0: 슬라이더 방향으로 y_offset_mm 만큼 당겨서 배치.

    알약 앞면 Y = WALL_Y_MM - th - y_offset_mm
    알약 뒷면 Y = WALL_Y_MM      - y_offset_mm
    """
    tau      = _density_tau(density)
    dimp_max = float(np.interp(density,
                               [DENSITY_REF_SOFT, DENSITY_REF_HARD],
                               [0.950, 0.999]))

    pos_x = PLACE_X_MM * 1e-3
    pos_z = PLACE_Z_MM * 1e-3
    pos_y = (WALL_Y_MM - half_th - y_offset_mm) * 1e-3  # 알약 중심 Y

    tree = ET.parse(MJCF_PATH)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler"); root.insert(0, compiler)
    compiler.set("meshdir", MJCF_DIR)
    for kf in root.findall("keyframe"):
        root.remove(kf)

    asset = root.find("asset")
    ET.SubElement(asset, "mesh",
                  {"name": "tablet_mesh", "file": "tablet.stl", "scale": ".001 .001 .001"})
    ET.SubElement(asset, "material",
                  {"name": "tablet_mat", "rgba": ".85 .80 .72 1"})

    wb  = root.find("worldbody")
    tab = ET.SubElement(wb, "body", {
        "name":  "tablet",
        "mocap": "true",
        "pos":   f"{pos_x:.6f} {pos_y:.6f} {pos_z:.6f}",
        "quat":  " ".join(f"{v:.7f}" for v in _TAB_QUAT),
    })
    ET.SubElement(tab, "geom", {
        "name":     "tablet_geom",
        "type":     "mesh",
        "mesh":     "tablet_mesh",
        "material": "tablet_mat",
        "density":  f"{density:.1f}",
        "condim":   "4",
        "friction": ".5 .02 .01",
        "solref":   f"{tau:.6f} 1",
        "solimp":   f"0.90 {dimp_max:.4f} 0.001",
    })

    xml_str   = ET.tostring(root, encoding="unicode")
    stl_bytes = open(stl_path, "rb").read()
    model     = mujoco.MjModel.from_xml_string(xml_str,
                                               assets={"tablet.stl": stl_bytes})
    return model, (pos_x, pos_y, pos_z)


def _slider_tablet_force_N(model, data, bid_slider, bid_tablet):
    """
    슬라이더 body ↔ 알약 body 접촉쌍만 분리해서 법선 압축력 합 [N].
    mocap 알약이므로 벽 접촉 제외하면 이 값 = 순수 압축력.
    """
    total_N = 0.0
    force6  = np.zeros(6)
    for i in range(data.ncon):
        c    = data.contact[i]
        g1_b = model.geom_bodyid[c.geom1]
        g2_b = model.geom_bodyid[c.geom2]
        if bid_slider not in (g1_b, g2_b):
            continue
        if bid_tablet not in (g1_b, g2_b):
            continue
        mujoco.mj_contactForce(model, data, i, force6)
        total_N += force6[0]   # 법선력 크기 (항상 ≥ 0)
    return total_N


# ─────────────────────────────────────────────────────────────────────
def run_one(stl_path, R_mm, AR, CV, y_offset_mm, density=DENSITY):
    """단일 y_offset 헤드리스 시뮬레이션."""
    cd      = CV * 2 * R_mm
    th      = R_mm * 0.20 + 2 * cd
    half_th = th / 2.0

    model, (px, py, pz) = _build_model(stl_path, half_th, y_offset_mm, density)
    data = mujoco.MjData(model)
    _dt  = float(model.opt.timestep)

    # ID 조회
    crank_jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                   "L3_Bevel_GearBox_1_L4_Shaft_1")
    act_crank  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                   "Motor1_crank")
    bid_tablet = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "tablet")
    bid_slider = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                   "L8_Link3_Shaft_1")
    eq_lock    = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                   "lock_crank")
    mid_tab    = model.body_mocapid[bid_tablet]
    crank_qadr = model.jnt_qposadr[crank_jid]
    crank_vadr = model.jnt_dofadr[crank_jid]

    # 초기 상태
    data.qpos[crank_qadr]    = -math.pi / 2
    data.qvel[:]             = 0.0
    data.mocap_pos[mid_tab]  = [px, py, pz]
    data.mocap_quat[mid_tab] = _TAB_QUAT
    data.ctrl[act_crank]     = 0.0
    mujoco.mj_forward(model, data)

    # Phase 1: 안정화
    for _ in range(PHASE1_STEPS):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    # Phase 2: 모터 ON
    p2_start  = data.time
    motor_on  = False

    t_log   = []
    f_log   = []   # 슬라이더-알약 압축력
    ang_log = []   # 크랭크 각도 [°]

    first_contact_t     = None
    first_contact_angle = None

    total_steps = int(SIM_DURATION / _dt)
    for step in range(total_steps):
        if not motor_on and (data.time - p2_start) >= MOTOR_DELAY:
            data.eq_active[eq_lock] = 0
            data.ctrl[act_crank]    = MOTOR_CTRL
            motor_on = True

        mujoco.mj_step(model, data)

        f_now   = _slider_tablet_force_N(model, data, bid_slider, bid_tablet)
        ang_now = math.degrees(data.qpos[crank_qadr])

        t_log.append(data.time)
        f_log.append(f_now)
        ang_log.append(ang_now)

        # 첫 접촉 기록
        if first_contact_t is None and f_now > 0.5:
            first_contact_t     = data.time
            first_contact_angle = ang_now

    t   = np.array(t_log)
    f   = np.array(f_log)
    ang = np.array(ang_log)

    F_max = float(f.max()) if len(f) > 0 else 0.0
    J     = float(_np_trapz(f, t))
    F_max_angle = float(ang[int(np.argmax(f))]) if F_max > 0 else None

    return {
        "y_offset_mm":          y_offset_mm,
        "tablet_front_y_mm":    WALL_Y_MM - th - y_offset_mm,
        "F_max":                F_max,
        "J":                    J,
        "first_contact_t":      first_contact_t,
        "first_contact_angle":  first_contact_angle,
        "F_max_angle":          F_max_angle,
        "t":  t,
        "f":  f,
        "ang": ang,
    }


# ─────────────────────────────────────────────────────────────────────
def plot_results(results, R_mm, AR, CV):
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = f"tdp_R{R_mm:.1f}_AR{AR:.2f}_CV{CV:.2f}__{ts}"
    os.makedirs(RESULT_DIR, exist_ok=True)

    valid = [r for r in results if r["F_max"] > 0.1]
    if not valid:
        print("  [!] 유효한 접촉 결과 없음"); return

    n      = len(valid)
    cmap   = plt.cm.coolwarm
    colors = [cmap(i / max(n - 1, 1)) for i in range(n)]

    offsets     = [r["y_offset_mm"]         for r in valid]
    f_maxes     = [r["F_max"]               for r in valid]
    impulses    = [r["J"]                   for r in valid]
    theta_first = [r["first_contact_angle"] for r in valid]
    theta_fmax  = [r["F_max_angle"]         for r in valid]
    front_y     = [r["tablet_front_y_mm"]   for r in valid]

    # ── Fig 1: F 시계열 (모든 offset 오버레이) ────────────────────────
    fig1, ax1 = plt.subplots(figsize=(12, 5))
    for r, col in zip(valid, colors):
        ax1.plot(r["t"], r["f"], lw=1.5, color=col,
                 label=f"{r['y_offset_mm']:.0f}mm  (F_max={r['F_max']:.1f}N)")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("F_slider-tablet [N]  (슬라이더-알약 압축력)")
    ax1.set_title(f"y_offset별 슬라이더-알약 접촉력  |  R={R_mm:.1f} AR={AR:.2f} CV={CV:.2f}\n"
                  f"파란색 = 알약 벽 근처, 빨간색 = 슬라이더 가까이 (offset 큼)",
                  fontsize=10)
    ax1.legend(fontsize=8, ncol=3, loc="upper right")
    ax1.grid(True, alpha=0.3)
    fig1.tight_layout()
    p1 = os.path.join(RESULT_DIR, f"{stem}_01_timeseries.png")
    fig1.savefig(p1, dpi=130); plt.close(fig1)
    print(f"  → {p1}")

    # ── Fig 2: F_max & J vs y_offset ─────────────────────────────────
    fig2, axes2 = plt.subplots(1, 2, figsize=(12, 5))

    axes2[0].plot(offsets, f_maxes, 'o-', color="tab:red", lw=2, ms=8)
    for x, y in zip(offsets, f_maxes):
        axes2[0].annotate(f"{y:.1f}N", (x, y),
                          textcoords="offset points", xytext=(0, 8),
                          ha="center", fontsize=8)
    axes2[0].set_xlabel("y_offset [mm]  (알약을 슬라이더 방향으로 당긴 거리)")
    axes2[0].set_ylabel("F_max [N]")
    axes2[0].set_title("F_max vs y_offset\n"
                        "TDP 가설: offset 증가 → F_max 감소", fontsize=10)
    axes2[0].grid(True, alpha=0.3)

    axes2[1].plot(offsets, impulses, 's-', color="tab:blue", lw=2, ms=8)
    for x, y in zip(offsets, impulses):
        axes2[1].annotate(f"{y:.3f}", (x, y),
                          textcoords="offset points", xytext=(0, 8),
                          ha="center", fontsize=8)
    axes2[1].set_xlabel("y_offset [mm]")
    axes2[1].set_ylabel("충격량 J [N·s]")
    axes2[1].set_title("충격량 vs y_offset", fontsize=10)
    axes2[1].grid(True, alpha=0.3)

    fig2.tight_layout()
    p2 = os.path.join(RESULT_DIR, f"{stem}_02_fmax_vs_offset.png")
    fig2.savefig(p2, dpi=130); plt.close(fig2)
    print(f"  → {p2}")

    # ── Fig 3: 크랭크 각도 관계 (TDP 가설 핵심 검증) ─────────────────
    has_ang = all(a is not None for a in theta_first)
    if has_ang:
        fig3, axes3 = plt.subplots(1, 2, figsize=(13, 5))

        # 3-0: θ_contact vs y_offset
        axes3[0].plot(offsets, theta_first, 'o-', color="tab:purple", lw=2, ms=8)
        for x, y in zip(offsets, theta_first):
            axes3[0].annotate(f"{y:.1f}°", (x, y),
                              textcoords="offset points", xytext=(0, 8),
                              ha="center", fontsize=8)
        axes3[0].set_xlabel("y_offset [mm]")
        axes3[0].set_ylabel("크랭크 각도 @ 첫 접촉 [°]")
        axes3[0].set_title("접촉 발생 크랭크 각도 vs y_offset\n"
                            "offset 증가 → 더 이른 각도에서 접촉 (상사점에서 멀어짐?)",
                            fontsize=10)
        axes3[0].grid(True, alpha=0.3)

        # 3-1: F_max vs θ_contact — TDP 가설 직접 검증
        sc = axes3[1].scatter(theta_first, f_maxes, c=offsets,
                              cmap="coolwarm", s=150, zorder=5, edgecolors="k", lw=0.5)
        axes3[1].plot(theta_first, f_maxes, '--', color="gray", lw=1, alpha=0.5)
        for θ, F, off in zip(theta_first, f_maxes, offsets):
            axes3[1].annotate(f"{off:.0f}mm", (θ, F),
                              textcoords="offset points", xytext=(6, 4), fontsize=8)
        axes3[1].set_xlabel("크랭크 각도 @ 첫 접촉 [°]")
        axes3[1].set_ylabel("F_max [N]")
        axes3[1].set_title("★ TDP 가설 검증\nF_max vs 접촉 각도  (색=y_offset)",
                            fontsize=10, fontweight="bold")
        cb = fig3.colorbar(sc, ax=axes3[1], label="y_offset [mm]")
        axes3[1].grid(True, alpha=0.3)

        txt = ("TDP 가설이 맞으면:\n"
               "  상사점(0° 또는 180°)에 가까운 각도에서 접촉\n"
               "  → F_max 가 더 크게 나타남")
        axes3[1].text(0.03, 0.97, txt, transform=axes3[1].transAxes,
                      fontsize=8, va="top",
                      bbox=dict(boxstyle="round", facecolor="lightyellow",
                                edgecolor="gray", alpha=0.9))

        fig3.tight_layout()
        p3 = os.path.join(RESULT_DIR, f"{stem}_03_tdp_verification.png")
        fig3.savefig(p3, dpi=130); plt.close(fig3)
        print(f"  → {p3}")

    # ── Fig 4: 알약 앞면 Y 위치별 F_max 바 차트 ─────────────────────
    fig4, ax4 = plt.subplots(figsize=(10, 5))
    bars = ax4.bar(range(n), f_maxes, color=colors, edgecolor="k", linewidth=0.5)
    ax4.set_xticks(range(n))
    ax4.set_xticklabels(
        [f"offset={o:.0f}mm\nY_face={y:.1f}mm"
         for o, y in zip(offsets, front_y)], fontsize=8)
    ax4.set_ylabel("F_max [N]")
    ax4.set_title(f"알약 위치별 최대 압축력  |  R={R_mm:.1f} AR={AR:.2f} CV={CV:.2f}\n"
                  f"Y_face = 알약 앞면(슬라이더 접촉면) Y 좌표 [mm]", fontsize=10)
    for bar, f in zip(bars, f_maxes):
        ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
                 f"{f:.1f}N", ha="center", va="bottom", fontsize=9, fontweight="bold")
    ax4.grid(True, alpha=0.3, axis="y")
    fig4.tight_layout()
    p4 = os.path.join(RESULT_DIR, f"{stem}_04_bar.png")
    fig4.savefig(p4, dpi=130); plt.close(fig4)
    print(f"  → {p4}")

    # ── 텍스트 요약 ──────────────────────────────────────────────────
    print(f"\n  {'offset':>8} | {'앞면Y':>8} | {'F_max':>8} | {'J':>8} | {'θ_contact':>10} | {'θ_Fmax':>8}")
    print("  " + "─" * 65)
    for r in valid:
        θc = f"{r['first_contact_angle']:>8.1f}°" if r['first_contact_angle'] else "     N/A"
        θm = f"{r['F_max_angle']:>6.1f}°"         if r['F_max_angle']         else "   N/A"
        print(f"  {r['y_offset_mm']:>7.0f}mm | "
              f"{r['tablet_front_y_mm']:>6.2f}mm | "
              f"{r['F_max']:>7.2f}N | "
              f"{r['J']:>8.4f} | "
              f"{θc} | {θm}")


# ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="상사점(TDP) 위치 가설 검증 — 알약 Y 위치 스위프")
    parser.add_argument("--stl",     type=str,   default=None,
                        help="알약 STL 파일 (기본: STL 폴더 첫 번째 파일)")
    parser.add_argument("--R",       type=float, default=4.0)
    parser.add_argument("--AR",      type=float, default=1.00)
    parser.add_argument("--CV",      type=float, default=0.20)
    parser.add_argument("--offsets", type=float, nargs="+",
                        default=DEFAULT_OFFSETS,
                        help="y_offset 목록 [mm]  (기본: 0~30mm)")
    parser.add_argument("--density", type=float, default=DENSITY)
    args = parser.parse_args()

    global mujoco
    try:
        import mujoco
    except ImportError as e:
        print(f"[ERROR] mujoco import 실패: {e}")
        print("  conda activate isaac_sim")
        sys.exit(1)

    # STL 파일 결정
    if args.stl:
        stl_path = args.stl if os.path.isabs(args.stl) else \
                   os.path.join(STL_DIR, args.stl)
    else:
        pat = os.path.join(STL_DIR,
              f"tablet_R{args.R:.1f}_AR{args.AR:.2f}_CV{args.CV:.2f}.stl")
        candidates = sorted(glob.glob(pat))
        if not candidates:
            # 폴더 첫 번째 STL 시도
            candidates = sorted(glob.glob(os.path.join(STL_DIR, "*.stl")))
        if not candidates:
            print(f"[ERROR] STL 없음: {STL_DIR}"); sys.exit(1)
        stl_path = candidates[0]
        print(f"[Auto STL] {os.path.basename(stl_path)}")

    if not os.path.exists(stl_path):
        print(f"[ERROR] 파일 없음: {stl_path}"); sys.exit(1)

    R_mm, AR, CV = _parse_params(stl_path)
    if R_mm is None:
        print("[ERROR] 파일명에서 R/AR/CV 파싱 실패"); sys.exit(1)

    cd  = CV * 2 * R_mm
    th  = R_mm * 0.20 + 2 * cd

    print(f"\n{'='*62}")
    print(f"  TDP 위치 가설 검증")
    print(f"  STL     : {os.path.basename(stl_path)}")
    print(f"  R={R_mm:.1f}mm  AR={AR:.2f}  CV={CV:.2f}  두께={th:.2f}mm")
    print(f"  벽 위치 : Y = {WALL_Y_MM:.2f} mm")
    print(f"  y_offsets: {args.offsets} mm")
    print(f"  sim_dur  : {SIM_DURATION}s × {len(args.offsets)} = "
          f"{SIM_DURATION * len(args.offsets):.0f}s 예상")
    print(f"{'='*62}\n")

    results = []
    for i, offset in enumerate(args.offsets):
        front_y = WALL_Y_MM - th - offset
        print(f"[{i+1}/{len(args.offsets)}] y_offset={offset:.0f}mm  "
              f"알약 앞면 Y={front_y:.2f}mm", end="  ", flush=True)
        r = run_one(stl_path, R_mm, AR, CV, offset, args.density)
        θ = f"{r['first_contact_angle']:.1f}°" if r['first_contact_angle'] else "N/A"
        print(f"→ F_max={r['F_max']:.2f}N  J={r['J']:.4f}N·s  θ_contact={θ}")
        results.append(r)

    print(f"\n[그래프 저장 중...]")
    plot_results(results, R_mm, AR, CV)
    print("\n완료.")


if __name__ == "__main__":
    main()

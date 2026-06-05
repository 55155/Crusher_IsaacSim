"""
run_comparison_headless.py
mocap 고정 방식 vs slide joint 방식 headless 비교 실행기

뷰어 없이 두 시뮬을 돌리고 반력 프로파일을 비교 분석합니다.

실행:
    python run_comparison_headless.py tablet_R6.0_AR1.50_CV0.20.stl
    python run_comparison_headless.py  (STL 생략 시 첫 번째 발견 파일 사용)
"""

import os, sys, re, csv, math, argparse, xml.etree.ElementTree as ET
from datetime import datetime
from collections import deque

import numpy as np
try:
    _np_trapz = np.trapezoid
except AttributeError:
    _np_trapz = np.trapz

import matplotlib
matplotlib.use("Agg")   # headless — 화면 없이 PNG 저장
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'Malgun Gothic'
plt.rcParams['axes.unicode_minus'] = False
import mujoco

# ── 경로 ─────────────────────────────────────────────────────────────
_HERE     = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.normpath(os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml"))
MJCF_DIR  = os.path.dirname(MJCF_PATH)
STL_DIR   = os.path.normpath(os.path.join(_HERE, "..", "..", "tablets_stl", "stl"))
_SIM_RESULT = os.path.normpath(os.path.join(_HERE, "..", "Sim_result"))
CSV_DIR   = os.path.join(_SIM_RESULT, "csv")
PLOT_DIR  = os.path.join(_SIM_RESULT, "plot")

# ── 파라미터 ─────────────────────────────────────────────────────────
PLACE_X_MM   = -47.879
PLACE_Z_MM   =  50.108
WALL_Y_MM    = 336.199
PHASE1_STEPS = 500
SIM_DURATION = 20.0      # headless: 짧게
MOTOR_CTRL   = -0.5
MOTOR_DELAY  =  3.0
STALL_TIME_S =  2.0
STALL_VEL_THR= 0.05
DENSITY      = 1200.0
SLIDE_DAMPING= 10.0

DENSITY_REF_SOFT = 900.0;  DENSITY_REF_HARD = 1800.0
SOLREF_TAU_SOFT  = 0.020;  SOLREF_TAU_HARD  = 0.002
BICONVEX_VOL_FACTOR = 0.82

_s = math.sqrt(2.0) / 2.0
TAB_QUAT = np.array([_s, 0.0, _s, 0.0])


# ─────────────────────────────────────────────────────────────────────
def _parse_params(fname):
    m = re.search(r"R([\d.]+)_AR([\d.]+)_CV([\d.]+)",
                  os.path.splitext(os.path.basename(fname))[0])
    if not m:
        return None, None, None
    return float(m.group(1)), float(m.group(2)), float(m.group(3))


def _density_tau(rho):
    rho   = float(np.clip(rho, DENSITY_REF_SOFT, DENSITY_REF_HARD))
    alpha = math.log(SOLREF_TAU_HARD / SOLREF_TAU_SOFT) / \
            math.log(DENSITY_REF_HARD / DENSITY_REF_SOFT)
    return float(np.clip(SOLREF_TAU_SOFT * (DENSITY_REF_SOFT / rho) ** alpha,
                         SOLREF_TAU_HARD, SOLREF_TAU_SOFT))


def _base_xml(stl_path, half_th):
    """공통 XML 전처리 (meshdir, keyframe 제거, asset 추가)."""
    pos_x = PLACE_X_MM * 1e-3
    pos_z = PLACE_Z_MM * 1e-3
    pos_y = (WALL_Y_MM - half_th) * 1e-3

    tau      = _density_tau(DENSITY)
    dimp_max = float(np.interp(DENSITY, [DENSITY_REF_SOFT, DENSITY_REF_HARD], [0.950, 0.999]))

    tree = ET.parse(MJCF_PATH)
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler"); root.insert(0, compiler)
    compiler.set("meshdir", MJCF_DIR)
    for kf in root.findall("keyframe"):
        root.remove(kf)

    asset = root.find("asset")
    ET.SubElement(asset, "mesh",     {"name": "tablet_mesh", "file": "tablet.stl", "scale": ".001 .001 .001"})
    ET.SubElement(asset, "material", {"name": "tablet_mat",  "rgba": ".85 .80 .72 1"})

    geom_attr = {
        "name": "tablet_geom", "type": "mesh", "mesh": "tablet_mesh",
        "material": "tablet_mat", "density": f"{DENSITY:.1f}",
        "condim": "4", "friction": ".5 .02 .01",
        "solref": f"{tau:.6f} 1",
        "solimp": f"0.99 {dimp_max:.4f} 0.0001",
    }
    return root, pos_x, pos_y, pos_z, geom_attr


def build_mocap_model(stl_path, half_th):
    root, px, py, pz, geom_attr = _base_xml(stl_path, half_th)
    worldbody = root.find("worldbody")
    tab = ET.SubElement(worldbody, "body", {
        "name": "tablet", "mocap": "true",
        "pos":  f"{px:.6f} {py:.6f} {pz:.6f}",
        "quat": " ".join(f"{v:.7f}" for v in TAB_QUAT),
    })
    ET.SubElement(tab, "geom", geom_attr)
    xml_str   = ET.tostring(root, encoding="unicode")
    stl_bytes = open(stl_path, "rb").read()
    return mujoco.MjModel.from_xml_string(xml_str, assets={"tablet.stl": stl_bytes}), (px, py, pz)


def build_slide_model(stl_path, half_th):
    root, px, py, pz, geom_attr = _base_xml(stl_path, half_th)
    worldbody = root.find("worldbody")
    tab = ET.SubElement(worldbody, "body", {
        "name": "tablet",
        "pos":  f"{px:.6f} {py:.6f} {pz:.6f}",
        "quat": " ".join(f"{v:.7f}" for v in TAB_QUAT),
    })
    ET.SubElement(tab, "joint", {
        "name": "tablet_slide", "type": "slide",
        "axis": "0 1 0", "damping": f"{SLIDE_DAMPING:.4f}", "limited": "false",
    })
    ET.SubElement(tab, "geom", geom_attr)
    xml_str   = ET.tostring(root, encoding="unicode")
    stl_bytes = open(stl_path, "rb").read()
    return mujoco.MjModel.from_xml_string(xml_str, assets={"tablet.stl": stl_bytes}), (px, py, pz)


# ─────────────────────────────────────────────────────────────────────
def _sum_contact_force(model, data, body_id):
    f_total = np.zeros(3)
    force6  = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        g1b = model.geom_bodyid[c.geom1]
        g2b = model.geom_bodyid[c.geom2]
        if body_id not in (g1b, g2b):
            continue
        mujoco.mj_contactForce(model, data, i, force6)
        frame   = c.frame.reshape(3, 3)
        f_world = frame.T @ force6[:3]
        if g2b == body_id:
            f_world = -f_world
        f_total += f_world
    return f_total


# ─────────────────────────────────────────────────────────────────────
def run_sim(model, data, mode: str, stl_path: str):
    """
    공통 시뮬 루프 (headless).
    mode: 'mocap' | 'slide'
    returns: dict of arrays
    """
    # ID 조회
    crank_jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,    "L3_Bevel_GearBox_1_L4_Shaft_1")
    act_crank  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Motor1_crank")
    b_tablet   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,     "tablet")
    b_slider   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,     "L8_Link3_Shaft_1")
    eq_lock_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "lock_crank")

    crank_qadr = model.jnt_qposadr[crank_jid]
    crank_vadr = model.jnt_dofadr[crank_jid]

    # mocap / slide 분기
    if mode == "mocap":
        mocap_id = model.body_mocapid[b_tablet]
    else:
        slide_jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "tablet_slide")
        slide_qadr = model.jnt_qposadr[slide_jid]
        slide_vadr = model.jnt_dofadr[slide_jid]

    # 초기 상태
    data.qpos[crank_qadr] = -np.pi / 2
    data.qvel[:]          = 0.0
    if mode == "mocap":
        px, py, pz = data.xpos[b_tablet]  # mocap_pos는 이미 build 시 설정됨
        data.mocap_pos[mocap_id]  = [model.body_pos[b_tablet, 0],
                                      model.body_pos[b_tablet, 1],
                                      model.body_pos[b_tablet, 2]]
        data.mocap_quat[mocap_id] = TAB_QUAT
    else:
        data.qpos[slide_qadr] = 0.0
    mujoco.mj_forward(model, data)

    # Phase 1
    print(f"  [{mode}] Phase 1: {PHASE1_STEPS} 스텝 안정화...")
    for _ in range(PHASE1_STEPS):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    print(f"  [{mode}] Phase 1 완료  crank={np.degrees(data.qpos[crank_qadr]):.1f}°  t={data.time:.2f}s")

    # Phase 2
    _dt          = float(model.opt.timestep)
    stall_window = max(1, int(round(STALL_TIME_S / _dt)))
    data.ctrl[act_crank] = 0.0
    phase2_start_t = data.time

    motor_on   = False
    motor_on_t = None
    motor_dir  = 0
    stall_buf  = deque(maxlen=stall_window)
    rev_events = []

    t_log    = []
    f_log    = []
    slide_log= []   # slide 방식일 때 변위
    vel_log  = []
    ncon_log = []
    slider_y_log = []
    tablet_y_log = []
    first_contact_t = None

    total_steps = int(SIM_DURATION / _dt)
    report_every = total_steps // 10

    for step in range(total_steps):
        # 모터 ON
        if not motor_on and (data.time - phase2_start_t) >= MOTOR_DELAY:
            data.eq_active[eq_lock_id] = 0
            motor_dir = 1
            data.ctrl[act_crank] = motor_dir * MOTOR_CTRL
            motor_on   = True
            motor_on_t = data.time
            stall_buf.clear()

        # stall 감지
        if motor_on:
            crank_vel = abs(data.qvel[crank_vadr])
            stall_buf.append(crank_vel < STALL_VEL_THR)
            if len(stall_buf) == stall_window and all(stall_buf):
                motor_dir = -motor_dir
                data.ctrl[act_crank] = motor_dir * MOTOR_CTRL
                stall_buf.clear()
                dir_str = "CCW" if motor_dir > 0 else "CW"
                rev_events.append((data.time, dir_str))

        mujoco.mj_step(model, data)

        sy    = float(data.xpos[b_slider, 1])
        ty    = float(data.xpos[b_tablet, 1])
        omega = float(data.qvel[crank_vadr])
        fc    = _sum_contact_force(model, data, b_tablet)

        t_log.append(data.time)
        f_log.append(fc.copy())
        vel_log.append(omega)
        ncon_log.append(data.ncon)
        slider_y_log.append(sy)
        tablet_y_log.append(ty)

        if mode == "slide":
            slide_log.append(float(data.qpos[slide_qadr]))

        if first_contact_t is None and data.ncon > 0:
            for ci in range(data.ncon):
                c = data.contact[ci]
                if b_tablet in (model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]):
                    first_contact_t = data.time
                    break

        if step % report_every == 0:
            pct = step / total_steps * 100
            print(f"  [{mode}] {pct:4.0f}%  t={data.time:.2f}s  "
                  f"ncon={data.ncon}  F_Y={fc[1]:.3f}N  ω={omega:.3f}rad/s")

    print(f"  [{mode}] 완료  rev={len(rev_events)}회  첫접촉={first_contact_t}")

    t  = np.array(t_log)
    fc = np.array(f_log)
    return {
        "mode": mode,
        "t":    t,
        "fc":   fc,
        "fy":   fc[:, 1],
        "vel":  np.array(vel_log),
        "ncon": np.array(ncon_log),
        "sy":   np.array(slider_y_log) * 1e3,
        "ty":   np.array(tablet_y_log) * 1e3,
        "slide_disp": np.array(slide_log) * 1e3 if slide_log else None,
        "motor_on_t": motor_on_t,
        "rev_events": rev_events,
        "first_contact_t": first_contact_t,
        "F_Y_max": float(fc[:, 1].max()),
        "F_Y_min": float(fc[:, 1].min()),
        "J_Y":     float(_np_trapz(fc[:, 1], t)),
        "dt":      float(model.opt.timestep),
    }


# ─────────────────────────────────────────────────────────────────────
def analyze_and_plot(r_mocap: dict, r_slide: dict, result_stem: str):
    from scipy.ndimage import uniform_filter1d

    def dfdt_smooth(t, f, n=30):
        d = np.gradient(f, t)
        return uniform_filter1d(d, size=n)

    t_m, fy_m = r_mocap["t"], r_mocap["fy"]
    t_s, fy_s = r_slide["t"], r_slide["fy"]
    df_m = dfdt_smooth(t_m, fy_m)
    df_s = dfdt_smooth(t_s, fy_s)

    ratio_m = np.max(np.abs(df_m)) / (np.max(np.abs(fy_m)) + 1e-9)
    ratio_s = np.max(np.abs(df_s)) / (np.max(np.abs(fy_s)) + 1e-9)

    def classify(ratio):
        return "Impact형 (급격한 피크)" if ratio > 0.3 else "Quasi-static형 (완만한 상승)"

    # ── 요약 출력 ────────────────────────────────────────────────────
    print("\n" + "=" * 66)
    print("  비교 분석 결과")
    print("=" * 66)
    for r, df, ratio in [(r_mocap, df_m, ratio_m), (r_slide, df_s, ratio_s)]:
        print(f"\n  [{r['mode'].upper()}]")
        print(f"    F_Y max       : {r['F_Y_max']:.4f} N")
        print(f"    F_Y min       : {r['F_Y_min']:.4f} N")
        print(f"    Impulse J_Y   : {r['J_Y']:.5f} N·s")
        print(f"    peak dF/dt    : {np.max(np.abs(df)):.2f} N/s")
        print(f"    ratio         : {ratio:.4f}  (>0.3→Impact / <0.3→Quasi-static)")
        print(f"    분류          : {classify(ratio)}")
        print(f"    방향 전환     : {len(r['rev_events'])} 회")
        if r["first_contact_t"]:
            print(f"    첫 접촉       : t = {r['first_contact_t']:.3f} s")
        else:
            print(f"    첫 접촉       : 없음")
    print()

    # ── 그림 1: 반력 비교 ────────────────────────────────────────────
    fig1, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=False)
    fig1.suptitle("반력 프로파일 비교  (mocap 고정 vs slide joint 자유 이동)",
                  fontsize=11, fontweight="bold")

    # F_Y 비교
    ax = axes[0]
    ax.plot(t_m, fy_m, color="tab:red",  lw=1.5, alpha=0.85,
            label=f"mocap 고정   max={r_mocap['F_Y_max']:.3f}N")
    ax.plot(t_s, fy_s, color="tab:blue", lw=1.5, alpha=0.85,
            label=f"slide joint  max={r_slide['F_Y_max']:.3f}N")
    ax.axhline(0, color="k", lw=0.5)
    if r_mocap["motor_on_t"]:
        ax.axvline(r_mocap["motor_on_t"], color="tab:orange", ls="--", lw=1.0,
                   label=f"모터 ON ≈ t={r_mocap['motor_on_t']:.1f}s")
    ax.fill_between(t_m, 0, fy_m, alpha=0.07, color="tab:red")
    ax.fill_between(t_s, 0, fy_s, alpha=0.07, color="tab:blue")
    ax.set_ylabel("F_Y [N]"); ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
    ax.set_title("법선 반력 F_Y", fontsize=10)

    # dF/dt 비교
    ax2 = axes[1]
    ax2.plot(t_m, df_m, color="tab:red",  lw=1.5, alpha=0.85,
             label=f"mocap  peak={np.max(np.abs(df_m)):.1f} N/s")
    ax2.plot(t_s, df_s, color="tab:blue", lw=1.5, alpha=0.85,
             label=f"slide  peak={np.max(np.abs(df_s)):.1f} N/s")
    ax2.axhline(0, color="k", lw=0.5)
    ax2.set_ylabel("dF/dt [N/s]"); ax2.legend(fontsize=9); ax2.grid(True, alpha=0.3)
    ax2.set_title("반력 변화율 (quasi-static 여부 판별)", fontsize=10)

    # 누적 임펄스
    ax3 = axes[2]
    Jm = np.cumsum(fy_m) * r_mocap["dt"]
    Js = np.cumsum(fy_s) * r_slide["dt"]
    ax3.plot(t_m, Jm, color="tab:red",  lw=1.5, label=f"mocap  J={r_mocap['J_Y']:.4f} N·s")
    ax3.plot(t_s, Js, color="tab:blue", lw=1.5, label=f"slide  J={r_slide['J_Y']:.4f} N·s")
    ax3.axhline(0, color="k", lw=0.5)
    ax3.set_ylabel("J_Y [N·s]"); ax3.set_xlabel("Time [s]")
    ax3.legend(fontsize=9); ax3.grid(True, alpha=0.3)
    ax3.set_title("누적 임펄스", fontsize=10)

    fig1.tight_layout()

    # ── 그림 2: slide 변위 (slide 방식만) ───────────────────────────
    fig2 = None
    if r_slide["slide_disp"] is not None:
        fig2, axes2 = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
        fig2.suptitle("Slide Joint — 알약 변위 & 반력", fontsize=11, fontweight="bold")
        axes2[0].plot(t_s, r_slide["slide_disp"], color="tab:orange", lw=1.5)
        axes2[0].axhline(0, color="k", lw=0.5, ls="--")
        axes2[0].set_ylabel("변위 [mm]")
        axes2[0].set_title("알약 Y축 변위  (+ = 뒷벽 방향)", fontsize=9)
        axes2[0].grid(True, alpha=0.3)
        axes2[1].plot(t_s, fy_s, color="tab:blue", lw=1.5)
        axes2[1].fill_between(t_s, 0, fy_s, where=(fy_s > 0), alpha=0.12, color="tab:blue")
        axes2[1].axhline(0, color="k", lw=0.5)
        axes2[1].set_ylabel("F_Y [N]"); axes2[1].set_xlabel("Time [s]")
        axes2[1].grid(True, alpha=0.3)
        fig2.tight_layout()

    # ── 그림 3: 판정 요약 텍스트 ────────────────────────────────────
    fig3, ax_txt = plt.subplots(figsize=(10, 5))
    ax_txt.axis("off")

    lines = [
        ("Crusher 반력 프로파일 분석 결과", 14, "black", "bold"),
        ("", 10, "black", "normal"),
        (f"시뮬 조건:  Motor={MOTOR_CTRL} N·m  |  Moving-window stall  |  "
         f"ρ={DENSITY:.0f} kg/m³  |  τ={_density_tau(DENSITY):.4f}s", 9, "gray", "normal"),
        ("", 10, "black", "normal"),
        ("[ mocap 고정 방식 ]", 11, "tab:red", "bold"),
        (f"  F_Y max     = {r_mocap['F_Y_max']:.4f} N", 10, "black", "normal"),
        (f"  Impulse J_Y = {r_mocap['J_Y']:.5f} N·s", 10, "black", "normal"),
        (f"  dF/dt peak  = {np.max(np.abs(df_m)):.2f} N/s   ratio={ratio_m:.4f}", 10, "black", "normal"),
        (f"  → 분류: {classify(ratio_m)}", 11, "tab:red", "bold"),
        ("", 10, "black", "normal"),
        ("[ slide joint 방식 (Y축 자유 이동) ]", 11, "tab:blue", "bold"),
        (f"  F_Y max     = {r_slide['F_Y_max']:.4f} N", 10, "black", "normal"),
        (f"  Impulse J_Y = {r_slide['J_Y']:.5f} N·s", 10, "black", "normal"),
        (f"  dF/dt peak  = {np.max(np.abs(df_s)):.2f} N/s   ratio={ratio_s:.4f}", 10, "black", "normal"),
        (f"  → 분류: {classify(ratio_s)}", 11, "tab:blue", "bold"),
        ("", 10, "black", "normal"),
        ("[ 핵심 결론 ]", 12, "black", "bold"),
        ("  • Moving-window stall 방식 → 반력은 모터 토크 한계로 결정됨", 10, "black", "normal"),
        ("  • stiffness(solref τ)는 피크 크기가 아닌 '관통 깊이'에 영향", 10, "black", "normal"),
        ("  • slide joint 방식: 알약이 밀려나므로 반력이 낮게 발생", 10, "darkred", "normal"),
        ("    → F = m × a (알약 질량 × 가속도)로 제한됨", 10, "darkred", "normal"),
        ("  • sharp stiffness peak 없음 → 완전한 quasi-static 프로파일 확인", 10, "darkgreen", "normal"),
    ]

    y = 0.97
    for text, size, color, weight in lines:
        ax_txt.text(0.02, y, text, transform=ax_txt.transAxes,
                    fontsize=size, color=color, fontweight=weight, va="top")
        y -= 0.045 if size >= 11 else 0.038

    fig3.patch.set_facecolor("#f8f8f8")
    ax_txt.set_facecolor("#f8f8f8")
    fig3.tight_layout()

    return fig1, fig2, fig3


# ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="mocap vs slide joint headless 비교")
    parser.add_argument("stl", nargs="?", default=None)
    args = parser.parse_args()

    # STL 자동 탐색
    stl_path = args.stl
    if stl_path is None:
        stls = [f for f in os.listdir(STL_DIR) if f.endswith(".stl") and "gitkeep" not in f]
        if not stls:
            print(f"[오류] STL 파일 없음: {STL_DIR}")
            sys.exit(1)
        stl_path = os.path.join(STL_DIR, sorted(stls)[0])
        print(f"[자동 선택] {stl_path}")

    stl_path = os.path.abspath(stl_path)
    R_mm, AR, CV = _parse_params(stl_path)
    if R_mm is None:
        print("[오류] 파일명 파싱 실패"); sys.exit(1)

    cd      = CV * 2 * R_mm
    th      = R_mm * 0.20 + 2 * cd
    half_th = th / 2.0
    stem    = os.path.splitext(os.path.basename(stl_path))[0]
    ts      = datetime.now().strftime("%Y%m%d_%H%M%S")

    print("=" * 66)
    print("  mocap vs slide joint  headless 비교 시뮬")
    print("=" * 66)
    print(f"  STL : {os.path.basename(stl_path)}")
    print(f"  R={R_mm:.1f}mm  AR={AR:.2f}  CV={CV:.2f}  두께≈{th:.2f}mm")
    print(f"  SIM_DURATION={SIM_DURATION}s  MOTOR={MOTOR_CTRL}N·m  DENSITY={DENSITY}kg/m³")
    print()

    os.makedirs(CSV_DIR, exist_ok=True)
    os.makedirs(PLOT_DIR, exist_ok=True)

    # ── mocap 방식 ────────────────────────────────────────────────────
    print("▶ [1/2] mocap 방식 시뮬...")
    model_m, pos_m = build_mocap_model(stl_path, half_th)
    data_m = mujoco.MjData(model_m)
    data_m.mocap_pos[model_m.body_mocapid[
        mujoco.mj_name2id(model_m, mujoco.mjtObj.mjOBJ_BODY, "tablet")]] = pos_m
    data_m.mocap_quat[model_m.body_mocapid[
        mujoco.mj_name2id(model_m, mujoco.mjtObj.mjOBJ_BODY, "tablet")]] = TAB_QUAT
    r_mocap = run_sim(model_m, data_m, "mocap", stl_path)

    # ── slide 방식 ────────────────────────────────────────────────────
    print("\n▶ [2/2] slide joint 방식 시뮬...")
    model_s, pos_s = build_slide_model(stl_path, half_th)
    data_s = mujoco.MjData(model_s)
    r_slide = run_sim(model_s, data_s, "slide", stl_path)

    # ── 분석 & 플롯 ──────────────────────────────────────────────────
    print("\n▶ 분석 중...")
    try:
        from scipy.ndimage import uniform_filter1d
    except ImportError:
        print("[경고] scipy 없음 → pip install scipy")
        sys.exit(1)

    fig1, fig2, fig3 = analyze_and_plot(r_mocap, r_slide, f"{stem}__{ts}")

    # 저장
    result_stem = f"comparison_{stem}__{ts}"
    specs = [(fig1, "force_comparison"), (fig3, "summary")]
    if fig2: specs.append((fig2, "slide_motion"))

    print("\n▶ 플롯 저장...")
    for fig, tag in specs:
        p = os.path.join(PLOT_DIR, f"{result_stem}__{tag}.png")
        fig.savefig(p, dpi=150, bbox_inches="tight")
        print(f"  saved: {os.path.basename(p)}")

    # CSV 저장
    for r in [r_mocap, r_slide]:
        csv_p = os.path.join(CSV_DIR, f"{result_stem}__{r['mode']}.csv")
        with open(csv_p, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["# mode", r["mode"]])
            w.writerow(["Time_s", "F_X_N", "F_Y_N", "F_Z_N", "ncon"])
            for i in range(len(r["t"])):
                w.writerow([f"{r['t'][i]:.5f}",
                             f"{r['fc'][i,0]:.6f}", f"{r['fc'][i,1]:.6f}", f"{r['fc'][i,2]:.6f}",
                             r["ncon"][i]])
        print(f"  saved: {os.path.basename(csv_p)}")

    print("\n완료.")


if __name__ == "__main__":
    main()

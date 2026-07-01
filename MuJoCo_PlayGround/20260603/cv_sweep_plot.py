"""
cv_sweep_plot.py
CV(곡률) 만 변화시키면서 Tablet 반력 F_Y 를 측정하는 헤드리스 배치 스윕

고정 파라미터: R=4.0 mm, AR=1.00, density=1200 kg/m³, RPM=8
변화 파라미터: CV (0.08 ~ 0.35, 스텝 0.03)

crusher_velocity_ctrl_viewer.py 의 물리 세팅과 완전히 동일하게 맞춤.

Usage:
    python cv_sweep_plot.py
    python cv_sweep_plot.py --R 6.0 --AR 1.50
    python cv_sweep_plot.py --density 1400 --duration 15
"""

import os, sys, re, math, argparse, xml.etree.ElementTree as ET

import numpy as np
try:
    _np_trapz = np.trapezoid
except AttributeError:
    _np_trapz = np.trapz

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import mujoco

# ── Paths ──────────────────────────────────────────────────────────────
_HERE       = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH   = os.path.normpath(os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml"))
MJCF_DIR    = os.path.dirname(MJCF_PATH)
STL_DIR     = os.path.normpath(os.path.join(_HERE, "..", "..", "tablets_stl", "stl"))
PLOT_DIR    = os.path.normpath(os.path.join(_HERE, "..", "Sim_result", "plot"))

# ── 고정 파라미터 (crusher_velocity_ctrl_viewer.py 와 동일) ────────────
PHASE1_STEPS   = 500
MOTOR_DELAY    = 2.0
TARGET_RPM     = 8.0

GEAR_RATIO       = 212.0
MOTOR_STALL_TORQ = 0.185
CRANK_R_M        = 0.020

_TAU_STALL_CRANK = 12.5
MOTOR_FORCELIM   = _TAU_STALL_CRANK
VEL_KV_DEFAULT   = _TAU_STALL_CRANK / (TARGET_RPM / 60.0 * 2 * math.pi)  # ≈ 14.9

DENSITY_DEFAULT  = 1200.0
_s       = math.sqrt(2.0) / 2.0
TAB_QUAT = np.array([_s, 0.0, _s, 0.0])

PLACE_X_MM = -47.879
PLACE_Z_MM =  50.108
WALL_Y_MM  = 336.199

# contact (viewer 와 동일)
SOLREF = "0.005 2"
SOLIMP = "0.99 0.999 0.001"

# ── 색상 팔레트 ────────────────────────────────────────────────────────
DARK  = '#12131a'
PANEL = '#1c1e2a'
TEXT  = '#dce0e8'
MUTED = '#7a8099'


# ── 헬퍼 ───────────────────────────────────────────────────────────────
def _build_model(stl_path, R_mm, half_th, density, kv, target_vel):
    pos_x = PLACE_X_MM * 1e-3
    pos_z = PLACE_Z_MM * 1e-3
    pos_y = (WALL_Y_MM - half_th) * 1e-3

    tree = ET.parse(MJCF_PATH)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler"); root.insert(0, compiler)
    compiler.set("meshdir", MJCF_DIR)
    for kf in root.findall("keyframe"):
        root.remove(kf)

    opt = root.find("option")
    if opt is None:
        opt = ET.Element("option"); root.insert(0, opt)
    opt.set("integrator", "implicitfast")
    opt.set("timestep",   "0.001")

    act_sec = root.find("actuator")
    for elem in list(act_sec):
        if elem.get("name") == "Motor1_crank":
            act_sec.remove(elem); break
    act_sec.insert(0, ET.Element("velocity", {
        "name":       "Motor1_crank",
        "joint":      "L3_Bevel_GearBox_1_L4_Shaft_1",
        "kv":         f"{kv:.1f}",
        "gear":       "1",
        "forcerange": f"{-MOTOR_FORCELIM:.1f} {MOTOR_FORCELIM:.1f}",
        "ctrlrange":  f"{-target_vel*2:.4f} {target_vel*2:.4f}",
    }))

    asset = root.find("asset")
    ET.SubElement(asset, "mesh",     {"name": "tablet_mesh", "file": "tablet.stl",
                                      "scale": ".001 .001 .001"})
    ET.SubElement(asset, "material", {"name": "tablet_mat",  "rgba": ".85 .80 .72 1"})
    wb  = root.find("worldbody")
    tab = ET.SubElement(wb, "body", {
        "name": "tablet", "mocap": "true",
        "pos":  f"{pos_x:.6f} {pos_y:.6f} {pos_z:.6f}",
        "quat": " ".join(f"{v:.7f}" for v in TAB_QUAT),
    })
    ET.SubElement(tab, "geom", {
        "name": "tablet_geom", "type": "mesh", "mesh": "tablet_mesh",
        "material": "tablet_mat", "density": f"{density:.1f}",
        "condim": "3", "friction": ".5 .02 .01",
        "solref": SOLREF,
        "solimp": SOLIMP,
    })

    stl_bytes = open(stl_path, "rb").read()
    xml_str   = ET.tostring(root, encoding="unicode")
    return mujoco.MjModel.from_xml_string(xml_str, assets={"tablet.stl": stl_bytes}), \
           (pos_x, pos_y, pos_z)


def _contact_force(model, data, body_id):
    f = np.zeros(3); f6 = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]
        if body_id not in (g1, g2): continue
        mujoco.mj_contactForce(model, data, i, f6)
        fw = c.frame.reshape(3, 3).T @ f6[:3]
        f += -fw if g2 == body_id else fw
    return f


def run_one(stl_path, density, duration):
    """단일 STL 헤드리스 실행 → (t, fy) 배열 반환."""
    target_vel = TARGET_RPM * 2.0 * math.pi / 60.0
    kv         = VEL_KV_DEFAULT

    fname = os.path.basename(stl_path)
    m = re.search(r"R([\d.]+)_AR([\d.]+)_CV([\d.]+)", os.path.splitext(fname)[0])
    R_mm, AR, CV = float(m.group(1)), float(m.group(2)), float(m.group(3))

    cd      = CV * 2 * R_mm
    th      = R_mm * 0.20 + 2 * cd
    half_th = th / 2.0

    model, (px, py, pz) = _build_model(stl_path, R_mm, half_th, density, kv, target_vel)
    data = mujoco.MjData(model)
    _dt  = float(model.opt.timestep)

    jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,    "L3_Bevel_GearBox_1_L4_Shaft_1")
    aid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Motor1_crank")
    bid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,     "tablet")
    eid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "lock_crank")
    mid  = model.body_mocapid[bid]
    qadr = model.jnt_qposadr[jid]
    vadr = model.jnt_dofadr[jid]

    data.qpos[qadr]       = -math.pi / 2
    data.qvel[:]          = 0.0
    data.mocap_pos[mid]   = [px, py, pz]
    data.mocap_quat[mid]  = TAB_QUAT
    data.ctrl[aid]        = 0.0
    mujoco.mj_forward(model, data)

    # Phase 1: settle
    for _ in range(PHASE1_STEPS):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    # Phase 2: motor ON
    p2_start   = data.time
    motor_on   = False
    total_steps = int(duration / _dt)
    t_log = []; fy_log = []

    for _ in range(total_steps):
        if not motor_on and (data.time - p2_start) >= MOTOR_DELAY:
            data.eq_active[eid] = 0
            data.ctrl[aid]      = target_vel
            motor_on = True

        mujoco.mj_step(model, data)
        fc = _contact_force(model, data, bid)
        t_log.append(data.time - p2_start - MOTOR_DELAY)   # motor ON 기준 0
        fy_log.append(fc[1])

    return np.array(t_log), np.array(fy_log), R_mm, AR, CV, th


# ── 메인 ───────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--R",        type=float, default=4.0)
    parser.add_argument("--AR",       type=float, default=1.00)
    parser.add_argument("--density",  type=float, default=DENSITY_DEFAULT)
    parser.add_argument("--duration", type=float, default=20.0)
    args = parser.parse_args()

    # 해당 R, AR 조합의 STL 파일 수집
    pattern = os.path.join(STL_DIR, f"tablet_R{args.R:.1f}_AR{args.AR:.2f}_CV*.stl")
    import glob
    stl_files = sorted(glob.glob(pattern))

    if not stl_files:
        print(f"[ERROR] STL not found: {pattern}"); sys.exit(1)

    print(f"  R={args.R} AR={args.AR}  density={args.density}  duration={args.duration}s")
    print(f"  {len(stl_files)} CV values: ", end="")

    cv_vals  = []
    results  = []

    for stl in stl_files:
        m = re.search(r"CV([\d]+\.[\d]+)", os.path.basename(stl))
        cv = float(m.group(1))
        cv_vals.append(cv)
        print(f"CV={cv:.2f} ", end="", flush=True)
        t, fy, R_mm, AR, CV, th = run_one(stl, args.density, args.duration)
        results.append((cv, t, fy, th))
    print("\n  Done.\n")

    # ── Plot ──────────────────────────────────────────────────────────
    os.makedirs(PLOT_DIR, exist_ok=True)

    n    = len(results)
    cmap = cm.get_cmap("plasma", n)
    colors = [cmap(i) for i in range(n)]

    fig, (ax_main, ax_peak) = plt.subplots(
        2, 1, figsize=(13, 9),
        gridspec_kw={"height_ratios": [3, 1], "hspace": 0.38},
        facecolor=DARK)

    # ── 상단: F_Y(t) 오버레이 ─────────────────────────────────────────
    ax_main.set_facecolor(PANEL)
    for sp in ax_main.spines.values(): sp.set_color('#333')

    motor_on_drawn = False
    fy_peaks = []

    for (cv, t, fy, th), color in zip(results, colors):
        ax_main.plot(t, fy, color=color, lw=1.6, alpha=0.88,
                     label=f"CV={cv:.2f}  (th={th:.2f}mm)")
        fy_peaks.append((cv, fy.max(), fy.min(), _np_trapz(np.abs(fy), t), th))

    ax_main.axhline(0, color="#555", lw=0.8)
    ax_main.set_xlabel("Time after motor ON [s]", color=MUTED, fontsize=10)
    ax_main.set_ylabel("Tablet F_Y  [N]", color=TEXT, fontsize=10)
    ax_main.set_title(
        f"Tablet Reaction Force  vs  Curvature (CV)\n"
        f"R={args.R:.1f} mm  |  AR={args.AR:.2f}  |  ρ={args.density:.0f} kg/m³  |  "
        f"{TARGET_RPM} RPM  |  τ_max={MOTOR_FORCELIM} N·m",
        color=TEXT, fontsize=11, fontweight="bold")
    ax_main.tick_params(colors=TEXT, labelsize=9)
    ax_main.grid(True, alpha=0.18, color="#444")

    # 컬러바 대용 범례 (CV 낮→높 순서)
    legend = ax_main.legend(fontsize=8, loc="upper right",
                             facecolor=PANEL, edgecolor="#555",
                             labelcolor=TEXT, ncol=2)

    # ── 하단: F_Y_max vs CV 바 차트 ───────────────────────────────────
    ax_peak.set_facecolor(PANEL)
    for sp in ax_peak.spines.values(): sp.set_color('#333')

    cvs_arr  = [r[0] for r in fy_peaks]
    fmax_arr = [r[1] for r in fy_peaks]
    fmin_arr = [r[2] for r in fy_peaks]
    imp_arr  = [r[3] for r in fy_peaks]

    bars = ax_peak.bar(cvs_arr, fmax_arr, width=0.022,
                       color=colors, edgecolor="#222", linewidth=0.8, alpha=0.90,
                       label="F_Y_max [N]")

    # 각 바에 수치 표시
    for bar, fmax in zip(bars, fmax_arr):
        ax_peak.text(bar.get_x() + bar.get_width()/2, fmax + fmax*0.02,
                     f"{fmax:.1f}", ha="center", va="bottom",
                     color=TEXT, fontsize=7.5, fontweight="bold")

    ax_peak.set_xlabel("CV  (curvature parameter)", color=MUTED, fontsize=10)
    ax_peak.set_ylabel("F_Y_max  [N]", color=TEXT, fontsize=10)
    ax_peak.set_title("Peak Reaction Force vs CV", color=TEXT, fontsize=10, fontweight="bold")
    ax_peak.set_xticks(cvs_arr)
    ax_peak.set_xticklabels([f"{c:.2f}" for c in cvs_arr], fontsize=8.5)
    ax_peak.tick_params(colors=TEXT, labelsize=9)
    ax_peak.grid(True, axis="y", alpha=0.18, color="#444")

    # 컬러바 (CV → 색상 매핑 직관적으로)
    sm = plt.cm.ScalarMappable(cmap="plasma",
                                norm=plt.Normalize(vmin=min(cvs_arr), vmax=max(cvs_arr)))
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax_peak, orientation="vertical",
                        fraction=0.03, pad=0.02)
    cbar.set_label("CV", color=TEXT, fontsize=9)
    cbar.ax.yaxis.set_tick_params(color=TEXT, labelcolor=TEXT)

    # 저장
    from datetime import datetime
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    fname_out = os.path.join(
        PLOT_DIR,
        f"cv_sweep_R{args.R:.1f}_AR{args.AR:.2f}_rho{args.density:.0f}__{ts}.png")
    fig.savefig(fname_out, dpi=150, bbox_inches="tight", facecolor=DARK)
    print(f"  Saved: {fname_out}")

    # 콘솔 요약
    print(f"\n  {'CV':>6}  {'th[mm]':>8}  {'F_max[N]':>10}  {'F_min[N]':>10}  {'|J|[N·s]':>10}")
    print("  " + "-"*50)
    for cv, fmax, fmin, imp, th in fy_peaks:
        print(f"  {cv:6.2f}  {th:8.2f}  {fmax:10.2f}  {fmin:10.2f}  {imp:10.4f}")


if __name__ == "__main__":
    main()

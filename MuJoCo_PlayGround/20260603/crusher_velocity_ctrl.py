"""
crusher_velocity_ctrl.py  [velocity control — 8 RPM]
Crusher + Tablet simulation

Control:
    motor -> velocity actuator  (8 RPM constant, quasi-static, forcelim=12.5 N*m)

Output (2 subplots):
    1. Crank speed [RPM]
    2. Tablet reaction force F_Y [N]

Usage:
    python crusher_velocity_ctrl.py [tablet.stl] [--rpm 8] [--kv 14.9] [--density 1200]
"""

import os, sys, re, csv, math, argparse, xml.etree.ElementTree as ET
from collections import deque
from datetime import datetime

import numpy as np
try:
    _np_trapz = np.trapezoid
except AttributeError:
    _np_trapz = np.trapz

import matplotlib
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer

# ── Paths ─────────────────────────────────────────────────────────────
_HERE     = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.normpath(os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml"))
MJCF_DIR  = os.path.dirname(MJCF_PATH)
STL_DIR   = os.path.normpath(os.path.join(_HERE, "..", "..", "tablets_stl", "stl"))
_SIM_RESULT = os.path.normpath(os.path.join(_HERE, "..", "Sim_result"))
CSV_DIR   = os.path.join(_SIM_RESULT, "csv")
PLOT_DIR  = os.path.join(_SIM_RESULT, "plot")

# ── Placement ─────────────────────────────────────────────────────────
PLACE_X_MM = -47.879
PLACE_Z_MM =  50.108
WALL_Y_MM  = 336.199

# ── Simulation params ─────────────────────────────────────────────────
PHASE1_STEPS   = 500
SIM_DURATION   = 40.0
MOTOR_DELAY    =  2.0
TARGET_RPM     =  8.0
CCW_REVS       =  1.0
CW_REVS        =  1.0

# ── Real motor: BL4281 + 감속기 1:212 (준정적 조건) ───────────────────
GEAR_RATIO       = 212.0
MOTOR_STALL_TORQ = 0.185           # [N·m]   BL4281 stall 토크
MOTOR_NOLOAD_RPM = 5800.0          # [RPM]   무부하 속도
MOTOR_INERTIA_KG = 72e-7           # [kg·m²] 로터 관성 (= 72 g·cm²)
CRANK_R_M        = 0.020           # [m]     크랭크 반경 (2 cm)
ROD_L_M          = 0.080           # [m]     커넥팅 로드 길이 (8 cm)

# 크랭크 축 환산 관성: J_motor × n² → 준정적 수치 안정 조건 핵심
# 안정 조건: kv·dt/J_eff = 14.9×0.002/0.3236 = 0.092 << 2  ✓
_J_REFL = MOTOR_INERTIA_KG * GEAR_RATIO ** 2   # 7.2e-6 × 212² = 0.3236 kg·m²

# 크랭크 출력 stall 토크 (실측 역산):
#   실측 F ≈ 625 N,  r = 0.02 m
#   τ = F·r = 625×0.02 = 12.5 N·m  (θ=90° 기준 — 최소 증폭 시 하한)
#   암시 감속기 효율 = 12.5 / (0.185×212) ≈ 32%  (고감속비 특성)
_TAU_STALL_CRANK = 12.5                                                # [N·m]

MOTOR_FORCELIM  = _TAU_STALL_CRANK                                     # 12.5 N·m
VEL_KV_DEFAULT  = _TAU_STALL_CRANK / (TARGET_RPM / 60.0 * 2 * math.pi)  # ≈ 14.9
# 의미: kv × ω_target = stall 토크  ← 속도=0(stall)에서 정격 토크 공급

# ── Real-motor stall detection (mirrors Keyborad_control_v2.py) ───────
# Real motor: rpm_buffer = deque(maxlen=5); stall when sum==0
# Simulation: angle-progress based — velocity actuator oscillates ±37 RPM when
#             blocked (RPM never drops to 0), so RPM threshold is unreliable.
#             Instead: if net crank progress in commanded direction < STALL_ANG_DEG
#             over STALL_TIME_S, treat as stall.
STALL_ANG_DEG  =  5.0    # [deg] min forward progress in STALL_TIME_S to avoid stall
STALL_TIME_S   =  0.5    # [s] observation window
# Real motor: time.sleep(0.5) between enable(0) and enable(1)
# Simulation: run SETTLE_TIME_S with ctrl=0 before re-applying velocity cmd
SETTLE_TIME_S  =  0.5    # [s] coast period after direction flip
STALL_DWELL_S  =  3.0    # [s] hold at stall before reversing
COOLDOWN_S     =  2.0    # [s] stall-detection cooldown after new direction applied

# ── Tablet ────────────────────────────────────────────────────────────
DENSITY_DEFAULT  = 1200.0
_s = math.sqrt(2.0) / 2.0
TAB_QUAT = np.array([_s, 0.0, _s, 0.0])


# ─────────────────────────────────────────────────────────────────────
def _parse_params(fname):
    m = re.search(r"R([\d.]+)_AR([\d.]+)_CV([\d.]+)",
                  os.path.splitext(os.path.basename(fname))[0])
    return (float(m.group(1)), float(m.group(2)), float(m.group(3))) if m else (None, None, None)


def _build_model(stl_path, R_mm, half_th, density_kg_m3, kv, target_vel):
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

    # ── 준정적 조건: implicitfast 적분기 적용 ─────────────────────────────
    # velocity actuator 는 explicit Euler 에서 kv·dt/J_eff < 2 를 요구하나,
    # 기구학 유효 관성이 너무 작아 kv=14.9 로 이 조건을 만족할 수 없음.
    # (kv·dt/J_mech ≈ 193 >> 2)
    # implicitfast: velocity/position actuator 힘을 암묵적으로 선형화
    # → kv 크기 제약 없이 안정 보장, J_REFL body 주입 불필요.
    opt = root.find("option")
    if opt is None:
        opt = ET.Element("option"); root.insert(0, opt)
    opt.set("integrator", "implicitfast")
    opt.set("timestep",   "0.001")

    # Replace motor -> velocity actuator
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

    # Tablet assets + mocap body
    asset = root.find("asset")
    ET.SubElement(asset, "mesh",     {"name": "tablet_mesh", "file": "tablet.stl",
                                      "scale": ".001 .001 .001"})
    ET.SubElement(asset, "material", {"name": "tablet_mat",  "rgba": ".85 .80 .72 1"})
    wb = root.find("worldbody")
    tab = ET.SubElement(wb, "body", {
        "name": "tablet", "mocap": "true",
        "pos":  f"{pos_x:.6f} {pos_y:.6f} {pos_z:.6f}",
        "quat": " ".join(f"{v:.7f}" for v in TAB_QUAT),
    })
    ET.SubElement(tab, "geom", {
        "name": "tablet_geom", "type": "mesh", "mesh": "tablet_mesh",
        "material": "tablet_mat", "density": f"{density_kg_m3:.1f}",
        "condim": "3", "friction": ".5 .02 .01",
        "solref": "0.001 2",
        "solimp": "0.99 0.999 0.001",
        "margin": "0.001",
    })

    xml_str   = ET.tostring(root, encoding="unicode")
    stl_bytes = open(stl_path, "rb").read()
    return mujoco.MjModel.from_xml_string(xml_str, assets={"tablet.stl": stl_bytes}), (pos_x, pos_y, pos_z)


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


# ─────────────────────────────────────────────────────────────────────
def run(stl_path, density_kg_m3=DENSITY_DEFAULT, kv=VEL_KV_DEFAULT, target_rpm=TARGET_RPM):
    target_vel = target_rpm * 2.0 * math.pi / 60.0

    print("=" * 62)
    print(f"  Crusher  [Velocity Control — {target_rpm} RPM]")
    print("=" * 62)

    fname = os.path.basename(stl_path)
    R_mm, AR, CV = _parse_params(fname)
    if R_mm is None:
        print("[ERROR] Filename parse failed"); sys.exit(1)

    cd      = CV * 2 * R_mm
    th      = R_mm * 0.20 + 2 * cd
    half_th = th / 2.0

    print(f"  STL    : {fname}")
    print(f"  R={R_mm:.1f} AR={AR:.2f} CV={CV:.2f}  thickness={th:.2f}mm")
    print(f"  Target : {target_rpm} RPM = {target_vel:.4f} rad/s")
    print(f"  kv={kv:.0f} N*m*s/rad  |  forcelim={MOTOR_FORCELIM:.0f} N*m\n")

    os.makedirs(CSV_DIR, exist_ok=True)
    os.makedirs(PLOT_DIR, exist_ok=True)
    ts          = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_stem = f"velctrl_{os.path.splitext(fname)[0]}__{ts}"

    model, (px, py, pz) = _build_model(stl_path, R_mm, half_th, density_kg_m3, kv, target_vel)
    data = mujoco.MjData(model)
    _dt  = float(model.opt.timestep)

    # IDs
    jid_crank  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,    "L3_Bevel_GearBox_1_L4_Shaft_1")
    aid_crank  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Motor1_crank")
    bid_tablet = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,     "tablet")
    bid_slider = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,     "L8_Link3_Shaft_1")
    eid_lock   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "lock_crank")
    mid_tab    = model.body_mocapid[bid_tablet]
    qadr       = model.jnt_qposadr[jid_crank]
    vadr       = model.jnt_dofadr[jid_crank]

    # Init
    data.qpos[qadr]    = -math.pi / 2
    data.qvel[:]       = 0.0
    data.mocap_pos[mid_tab]  = [px, py, pz]
    data.mocap_quat[mid_tab] = TAB_QUAT
    data.ctrl[aid_crank]     = 0.0
    mujoco.mj_forward(model, data)

    # Phase 1
    print(f"[Phase 1] {PHASE1_STEPS} steps ...")
    for _ in range(PHASE1_STEPS):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    print(f"  Done  crank={math.degrees(data.qpos[qadr]):.1f} deg\n")

    # Phase 2
    rev_angle = 2.0 * math.pi
    p2_start  = data.time
    motor_on  = False
    motor_on_t= None
    direction = 1
    prev_ang  = float(data.qpos[qadr])
    acc_ccw = acc_cw = 0.0
    rev_events = []

    # Stall / settle
    stall_window          = max(1, int(round(STALL_TIME_S  / _dt)))
    settle_steps          = max(1, int(round(SETTLE_TIME_S / _dt)))
    dwell_steps           = max(1, int(round(STALL_DWELL_S / _dt)))
    cooldown_steps        = max(1, int(round(COOLDOWN_S    / _dt)))
    angle_buf             = deque(maxlen=stall_window)
    settle_countdown      = 0
    stall_dwell_countdown = 0
    cooldown_countdown    = 0
    prev_rpm              = 0.0

    t_log = []; f_log = []; rpm_log = []; torq_log = []; ang_log = []

    print(f"[Phase 2] Viewer open — motor starts at t+{MOTOR_DELAY}s")
    print(f"  {'Time':>6s} | {'RPM':>7s} | {'Torque':>8s} N*m | {'F_Y':>8s} N | {'ncon':>4s}")
    print("  " + "-" * 55)

    # Realtime plot (2 subplots)
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    fig.suptitle(f"Crusher {target_rpm} RPM Velocity Control  |  kv={kv:.0f}  lim={MOTOR_FORCELIM:.0f} N*m",
                 fontsize=11, fontweight="bold")

    line_rpm,   = ax1.plot([], [], color="tab:purple", lw=1.5, label="Crank speed [RPM]")
    ax1.axhline( target_rpm, color="tab:purple", ls=":", lw=1.0, alpha=0.5,
                 label=f"Target {target_rpm} RPM")
    ax1.axhline(-target_rpm, color="tab:purple", ls=":", lw=1.0, alpha=0.5)
    ax1.axhline(0, color="gray", lw=0.5)
    ax1.set_ylabel("RPM")
    ax1b = ax1.twinx()
    line_ang, = ax1b.plot([], [], color="tab:green", lw=1.0, ls="-.", alpha=0.75,
                           label="Crank angle [°]")
    ax1b.set_ylabel("Crank angle [°]", color="tab:green")
    ax1b.tick_params(axis='y', labelcolor='tab:green')
    _l1, _lb1 = ax1.get_legend_handles_labels()
    _l2, _lb2 = ax1b.get_legend_handles_labels()
    ax1.legend(_l1 + _l2, _lb1 + _lb2, fontsize=9)
    ax1.grid(True, alpha=0.3)

    line_fy,  = ax2.plot([], [], color="tab:blue", lw=1.5, label="Tablet F_Y [N]")
    ax2.axhline(0, color="k", lw=0.5)
    ax2.set_ylabel("F_Y [N]"); ax2.set_xlabel("Time [s]")
    ax2.legend(fontsize=9); ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.canvas.draw(); fig.canvas.flush_events()
    _vlines = []

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT]      = False
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = False
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_NONE

        while viewer.is_running() and data.time < SIM_DURATION:

            # Motor ON
            if not motor_on and (data.time - p2_start) >= MOTOR_DELAY:
                data.eq_active[eid_lock] = 0
                direction = 1
                data.ctrl[aid_crank] = direction * target_vel
                motor_on = True; motor_on_t = data.time
                prev_ang = float(data.qpos[qadr])
                acc_ccw = acc_cw = 0.0
                angle_buf.clear(); settle_countdown = 0
                print(f"  *** Motor ON t={data.time:.2f}s  -> CCW {target_vel:.4f} rad/s ***")

            if motor_on:
                if settle_countdown > 0:
                    data.ctrl[aid_crank] = 0.0
                    settle_countdown -= 1
                    if settle_countdown == 0:
                        data.ctrl[aid_crank] = direction * target_vel
                        cooldown_countdown = cooldown_steps
                        angle_buf.clear()
                        prev_ang = float(data.qpos[qadr])
                        dir_s = "CCW" if direction == 1 else "CW"
                        print(f"  *** Settle done → {dir_s} ON  t={data.time:.2f}s ***")
                elif stall_dwell_countdown > 0:
                    # Keep pushing; wait for dwell window before reversing
                    stall_dwell_countdown -= 1
                    if stall_dwell_countdown == 0:
                        direction = -direction
                        data.ctrl[aid_crank] = 0.0
                        settle_countdown = settle_steps
                        acc_ccw = acc_cw = 0.0
                        angle_buf.clear()
                        dir_s = "CCW" if direction == 1 else "CW"
                        rev_events.append((data.time, dir_s))
                        print(f"  *** Dwell done → {dir_s} (settle {SETTLE_TIME_S:.1f}s)"
                              f"  t={data.time:.2f}s ***")
                else:
                    if cooldown_countdown > 0:
                        cooldown_countdown -= 1
                    else:
                        angle_buf.append(float(data.qpos[qadr]))
                        if len(angle_buf) == stall_window:
                            raw_delta = angle_buf[-1] - angle_buf[0]
                            raw_delta = (raw_delta + math.pi) % (2 * math.pi) - math.pi
                            net_progress = direction * raw_delta
                            if net_progress < math.radians(STALL_ANG_DEG):
                                stall_dwell_countdown = dwell_steps
                                print(f"  *** Stall detected → dwell {STALL_DWELL_S:.1f}s"
                                      f"  t={data.time:.2f}s ***")

                    # Angle-based reversal (fallback: free-run without tablet)
                    cur_ang = float(data.qpos[qadr])
                    delta   = ((cur_ang - prev_ang) + math.pi) % (2*math.pi) - math.pi
                    prev_ang = cur_ang
                    if direction == 1:
                        acc_ccw += max(delta, 0.0)
                        if acc_ccw >= CCW_REVS * rev_angle:
                            direction = -1
                            data.ctrl[aid_crank] = 0.0
                            settle_countdown = settle_steps
                            acc_ccw = acc_cw = 0.0
                            angle_buf.clear()
                            rev_events.append((data.time, "CW"))
                            print(f"  *** Angle→ CW (settle {SETTLE_TIME_S:.1f}s)"
                                  f"  t={data.time:.2f}s ***")
                    else:
                        acc_cw += max(-delta, 0.0)
                        if acc_cw >= CW_REVS * rev_angle:
                            direction = 1
                            data.ctrl[aid_crank] = 0.0
                            settle_countdown = settle_steps
                            acc_ccw = acc_cw = 0.0
                            angle_buf.clear()
                            rev_events.append((data.time, "CCW"))
                            print(f"  *** Angle→ CCW (settle {SETTLE_TIME_S:.1f}s)"
                                  f"  t={data.time:.2f}s ***")

            mujoco.mj_step(model, data)

            rpm_now  = float(data.qvel[vadr]) * 60.0 / (2.0 * math.pi)
            torq_now = float(data.actuator_force[aid_crank])
            fc_now   = _contact_force(model, data, bid_tablet)
            ang_now  = math.degrees(data.qpos[qadr])
            prev_rpm = rpm_now   # used by stall detection on next step

            t_log.append(data.time)
            rpm_log.append(rpm_now)
            torq_log.append(torq_now)
            f_log.append(fc_now.copy())
            ang_log.append(ang_now)

            if len(t_log) % 500 == 0:
                dir_s = "CCW" if direction == 1 else "CW"
                print(f"  {data.time:6.2f}s | {rpm_now:7.2f} | {torq_now:10.2f}    | "
                      f"{fc_now[1]:8.3f} | {data.ncon:4d}  [{dir_s}]")

            # Realtime plot update
            if len(t_log) % 20 == 0 and len(t_log) > 1:
                line_rpm.set_data(t_log, rpm_log)
                line_ang.set_data(t_log, ang_log)
                line_fy.set_data(t_log,  [f[1] for f in f_log])
                for ax in (ax1, ax1b, ax2):
                    ax.relim(); ax.autoscale_view()
                for vl in _vlines:
                    try: vl.remove()
                    except Exception: pass
                _vlines.clear()
                if motor_on_t:
                    for ax in (ax1, ax2):
                        _vlines.append(ax.axvline(motor_on_t, color="tab:green",
                                                   ls="--", lw=1.0, alpha=0.7))
                for ev_t, _ in rev_events:
                    for ax in (ax1, ax2):
                        _vlines.append(ax.axvline(ev_t, color="tab:red",
                                                   ls=":", lw=0.8, alpha=0.7))
                fig.canvas.draw_idle(); fig.canvas.flush_events()

            viewer.sync()

    plt.ioff()

    if not t_log:
        print("[WARNING] No data."); return

    t      = np.array(t_log)
    fc     = np.array(f_log)
    rpm    = np.array(rpm_log)
    torq   = np.array(torq_log)
    F_Y_max = float(fc[:, 1].max())
    F_Y_min = float(fc[:, 1].min())
    J_Y     = float(_np_trapz(fc[:, 1], t))

    print(f"\n  {'='*55}")
    print(f"  Steps      : {len(t)}  ({t[-1]:.2f} s)")
    print(f"  RPM range  : {rpm.min():.2f} ~ {rpm.max():.2f}  (target {target_rpm})")
    print(f"  Torque     : {torq.min():.2f} ~ {torq.max():.2f} N*m")
    print(f"  F_Y        : {F_Y_min:.3f} ~ {F_Y_max:.3f} N")
    print(f"  Impulse    : {J_Y:.5f} N*s")
    print(f"  Reversals  : {len(rev_events)}")
    print(f"  {'='*55}")

    # ── Final 2-subplot figure ────────────────────────────────────────
    fig2, (ax_a, ax_b) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig2.suptitle(
        f"Crusher {target_rpm} RPM  |  kv={kv:.0f}  forcelim={MOTOR_FORCELIM:.0f} N*m  |  "
        f"R={R_mm:.1f} AR={AR:.2f} CV={CV:.2f}  rho={density_kg_m3:.0f} kg/m3",
        fontsize=10, fontweight="bold")

    def _add_vlines(ax_):
        if motor_on_t:
            ax_.axvline(motor_on_t, color="tab:green", ls="--", lw=1.2,
                        label=f"Motor ON (t={motor_on_t:.1f}s)")
        for ev_t, ev_dir in rev_events:
            ax_.axvline(ev_t, color="tab:red", ls=":", lw=1.0, label=f"-> {ev_dir}")

    # Subplot 1: RPM + Crank angle
    ang = np.array(ang_log)
    ax_a.plot(t, rpm, color="tab:purple", lw=1.8, label="Crank speed [RPM]")
    ax_a.axhline( target_rpm, color="tab:purple", ls=":", lw=1.0, alpha=0.5,
                  label=f"Target {target_rpm} RPM")
    ax_a.axhline(-target_rpm, color="tab:purple", ls=":", lw=1.0, alpha=0.5)
    ax_a.axhline(0, color="gray", lw=0.5)
    _add_vlines(ax_a)
    ax_a.set_ylabel("RPM", fontsize=11)
    ax_aa = ax_a.twinx()
    ax_aa.plot(t, ang, color="tab:green", lw=1.0, ls="-.", alpha=0.7, label="Crank angle [°]")
    ax_aa.set_ylabel("Crank angle [°]", color="tab:green", fontsize=11)
    ax_aa.tick_params(axis='y', labelcolor='tab:green')
    _la, _lba = ax_a.get_legend_handles_labels()
    _laa, _lbaa = ax_aa.get_legend_handles_labels()
    ax_a.legend(_la + _laa, _lba + _lbaa, fontsize=9, loc="upper right")
    ax_a.set_title("Crank Speed & Angle", fontsize=10)
    ax_a.grid(True, alpha=0.3)

    # Subplot 2: Tablet F_Y
    ax_b.plot(t, fc[:, 1], color="tab:blue", lw=1.8, label="Tablet F_Y [N]")
    ax_b.fill_between(t, 0, fc[:, 1], where=(fc[:, 1] > 0),
                       alpha=0.15, color="tab:blue", label="compression")
    ax_b.fill_between(t, 0, fc[:, 1], where=(fc[:, 1] < 0),
                       alpha=0.15, color="tab:red", label="tension/rebound")
    ax_b.axhline(0, color="k", lw=0.5)
    _add_vlines(ax_b)
    ax_b.set_ylabel("F_Y [N]", fontsize=11)
    ax_b.set_xlabel("Time [s]", fontsize=11)
    ax_b.set_title(f"Tablet Reaction Force  |  max={F_Y_max:.2f} N  J={J_Y:.4f} N*s",
                   fontsize=10)
    ax_b.legend(fontsize=9, loc="upper right"); ax_b.grid(True, alpha=0.3)

    fig2.tight_layout()

    # Save
    p_rt  = os.path.join(PLOT_DIR, f"{result_stem}__realtime.png")
    p_out = os.path.join(PLOT_DIR, f"{result_stem}__result.png")
    csv_p = os.path.join(CSV_DIR,  f"{result_stem}.csv")

    fig.savefig(p_rt,  dpi=150, bbox_inches="tight")
    fig2.savefig(p_out, dpi=150, bbox_inches="tight")

    with open(csv_p, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["# Velocity Control — Tablet Force Profile"])
        w.writerow(["# STL", fname, "R_mm", R_mm, "AR", AR, "CV", CV])
        w.writerow(["# target_rpm", target_rpm, "kv", kv, "forcelim", MOTOR_FORCELIM])
        w.writerow(["# density_kg_m3", density_kg_m3])
        w.writerow(["# F_Y_max", f"{F_Y_max:.5f}", "F_Y_min", f"{F_Y_min:.5f}",
                    "Impulse_Ns", f"{J_Y:.6f}"])
        w.writerow([])
        w.writerow(["Time_s", "F_X_N", "F_Y_N", "F_Z_N", "Crank_RPM", "Motor_torque_Nm"])
        for i in range(len(t_log)):
            w.writerow([f"{t_log[i]:.5f}",
                        f"{f_log[i][0]:.5f}", f"{f_log[i][1]:.5f}", f"{f_log[i][2]:.5f}",
                        f"{rpm_log[i]:.4f}", f"{torq_log[i]:.4f}"])

    print(f"\n  Saved: {os.path.basename(p_out)}")
    print(f"  Saved: {os.path.basename(csv_p)}")
    plt.show()


# ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Crusher velocity control + tablet force")
    parser.add_argument("stl",       nargs="?", default=None)
    parser.add_argument("--rpm",     type=float, default=TARGET_RPM)
    parser.add_argument("--kv",      type=float, default=VEL_KV_DEFAULT)
    parser.add_argument("--density", type=float, default=DENSITY_DEFAULT)
    args = parser.parse_args()

    stl_path = args.stl
    if stl_path is None:
        try:
            import tkinter as tk
            from tkinter import filedialog
            root_tk = tk.Tk(); root_tk.withdraw()
            stl_path = filedialog.askopenfilename(
                title="Select Tablet STL",
                initialdir=STL_DIR if os.path.isdir(STL_DIR) else "~",
                filetypes=[("STL Files", "*.stl")],
            )
            root_tk.destroy()
        except Exception:
            pass

    if not stl_path:
        print("Usage: python crusher_velocity_ctrl.py <path>.stl [--rpm 8] [--kv 14.9]")
        sys.exit(0)

    stl_path = os.path.abspath(stl_path)
    if not os.path.exists(stl_path):
        print(f"[ERROR] File not found: {stl_path}"); sys.exit(1)

    run(stl_path, density_kg_m3=args.density, kv=args.kv, target_rpm=args.rpm)

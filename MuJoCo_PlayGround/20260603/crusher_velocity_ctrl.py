"""
crusher_velocity_ctrl.py  [velocity control — 8 RPM]
Crusher + Tablet simulation

Control:
    motor -> velocity actuator  (8 RPM constant, torque auto-adjusts up to 120 N*m)

Output (2 subplots):
    1. Crank speed [RPM] + Motor torque [N*m]
    2. Tablet reaction force F_Y [N]

Usage:
    python crusher_velocity_ctrl.py [tablet.stl] [--rpm 8] [--kv 60] [--density 1200]
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
MOTOR_FORCELIM =  3.0   # [N·m] 실제 소형 모터 stall 토크 수준 (이전 120→ 진동 원인)
VEL_KV_DEFAULT =  3.0   # [N·m·s/rad] kv×target_vel = stall 토크 ≈ 2.5 N·m (이전 60→ 과도한 게인)
CCW_REVS       =  1.0
CW_REVS        =  1.0

# ── Real-motor stall detection (mirrors Keyborad_control_v2.py) ───────
# Real motor: rpm_buffer = deque(maxlen=5); stall when sum==0
# Simulation: rpm_buf    = deque(maxlen=stall_window); stall when all < THR
STALL_RPM_THR  =  1.0    # [RPM] below this = stalled
STALL_TIME_S   =  0.5    # [s] sustained stall window (≈ 5 readings at real 10 Hz)
# Real motor: time.sleep(0.5) between enable(0) and enable(1)
# Simulation: run SETTLE_TIME_S with ctrl=0 before re-applying velocity cmd
SETTLE_TIME_S  =  0.5    # [s] coast period after direction flip

# ── Tablet ────────────────────────────────────────────────────────────
DENSITY_DEFAULT  = 1200.0
DENSITY_REF_SOFT = 900.0;  DENSITY_REF_HARD = 1800.0
SOLREF_TAU_SOFT  = 0.020;  SOLREF_TAU_HARD  = 0.002
_s = math.sqrt(2.0) / 2.0
TAB_QUAT = np.array([_s, 0.0, _s, 0.0])


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


def _build_model(stl_path, R_mm, half_th, density_kg_m3, kv, target_vel):
    pos_x = PLACE_X_MM * 1e-3
    pos_z = PLACE_Z_MM * 1e-3
    pos_y = (WALL_Y_MM - half_th) * 1e-3

    tau      = _density_tau(density_kg_m3)
    dimp_max = float(np.interp(density_kg_m3,
                               [DENSITY_REF_SOFT, DENSITY_REF_HARD], [0.950, 0.999]))

    tree = ET.parse(MJCF_PATH)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler"); root.insert(0, compiler)
    compiler.set("meshdir", MJCF_DIR)
    for kf in root.findall("keyframe"):
        root.remove(kf)

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
        "condim": "4", "friction": ".5 .02 .01",
        "solref": f"{tau:.6f} 1",
        "solimp": f"0.99 {dimp_max:.4f} 0.0001",
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

    # Stall / settle (mirrors Keyborad_control_v2.py deque logic)
    stall_window     = max(1, int(round(STALL_TIME_S  / _dt)))
    settle_steps     = max(1, int(round(SETTLE_TIME_S / _dt)))
    rpm_buf          = deque(maxlen=stall_window)  # True = below threshold
    settle_countdown = 0   # >0 while coasting (ctrl=0 between enable OFF/ON)
    prev_rpm         = 0.0

    t_log = []; f_log = []; rpm_log = []; torq_log = []

    print(f"[Phase 2] Viewer open — motor starts at t+{MOTOR_DELAY}s")
    print(f"  {'Time':>6s} | {'RPM':>7s} | {'Torque':>8s} N*m | {'F_Y':>8s} N | {'ncon':>4s}")
    print("  " + "-" * 55)

    # Realtime plot (2 subplots)
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    fig.suptitle(f"Crusher {target_rpm} RPM Velocity Control  |  kv={kv:.0f}  lim={MOTOR_FORCELIM:.0f} N*m",
                 fontsize=11, fontweight="bold")

    line_rpm,   = ax1.plot([], [], color="tab:purple", lw=1.5, label="Crank speed [RPM]")
    line_torq,  = ax1.plot([], [], color="tab:orange", lw=1.2, ls="--", alpha=0.8,
                            label="Motor torque [N*m]")
    ax1.axhline( target_rpm, color="tab:purple", ls=":", lw=1.0, alpha=0.5,
                 label=f"Target {target_rpm} RPM")
    ax1.axhline(-target_rpm, color="tab:purple", ls=":", lw=1.0, alpha=0.5)
    ax1.axhline( MOTOR_FORCELIM, color="tab:orange", ls=":", lw=0.8, alpha=0.5)
    ax1.axhline(-MOTOR_FORCELIM, color="tab:orange", ls=":", lw=0.8, alpha=0.5)
    ax1.axhline(0, color="gray", lw=0.5)
    ax1.set_ylabel("RPM  /  N*m")
    ax1.legend(fontsize=9); ax1.grid(True, alpha=0.3)

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
                rpm_buf.clear(); settle_countdown = 0
                print(f"  *** Motor ON t={data.time:.2f}s  -> CCW {target_vel:.4f} rad/s ***")

            if motor_on:
                # ── Settle period: ctrl=0 (mirrors set_enable(0)→sleep→set_enable(1)) ──
                if settle_countdown > 0:
                    data.ctrl[aid_crank] = 0.0
                    settle_countdown -= 1
                    if settle_countdown == 0:
                        # enable(1) equivalent — apply new direction velocity
                        data.ctrl[aid_crank] = direction * target_vel
                        rpm_buf.clear()
                        dir_s = "CCW" if direction == 1 else "CW"
                        print(f"  *** Settle done → {dir_s} ON  t={data.time:.2f}s ***")
                else:
                    # ── Stall detection (mirrors: sum(rpm_buffer)==0 in real motor) ──
                    rpm_buf.append(abs(prev_rpm) < STALL_RPM_THR)
                    if len(rpm_buf) == stall_window and all(rpm_buf):
                        # Stall → enable(0) → flip → settle → enable(1)
                        direction = -direction
                        data.ctrl[aid_crank] = 0.0          # enable OFF
                        settle_countdown = settle_steps
                        acc_ccw = acc_cw = 0.0
                        rpm_buf.clear()
                        dir_s = "CCW" if direction == 1 else "CW"
                        rev_events.append((data.time, dir_s))
                        print(f"  *** Stall → {dir_s} (settle {SETTLE_TIME_S:.1f}s)"
                              f"  t={data.time:.2f}s ***")
                    else:
                        # ── Angle-based reversal (fallback: free-run without tablet) ──
                        cur_ang = float(data.qpos[qadr])
                        delta   = ((cur_ang - prev_ang) + math.pi) % (2*math.pi) - math.pi
                        prev_ang = cur_ang
                        if direction == 1:
                            acc_ccw += max(delta, 0.0)
                            if acc_ccw >= CCW_REVS * rev_angle:
                                direction = -1
                                data.ctrl[aid_crank] = 0.0   # enable OFF
                                settle_countdown = settle_steps
                                acc_ccw = acc_cw = 0.0
                                rpm_buf.clear()
                                rev_events.append((data.time, "CW"))
                                print(f"  *** Angle→ CW (settle {SETTLE_TIME_S:.1f}s)"
                                      f"  t={data.time:.2f}s ***")
                        else:
                            acc_cw += max(-delta, 0.0)
                            if acc_cw >= CW_REVS * rev_angle:
                                direction = 1
                                data.ctrl[aid_crank] = 0.0   # enable OFF
                                settle_countdown = settle_steps
                                acc_ccw = acc_cw = 0.0
                                rpm_buf.clear()
                                rev_events.append((data.time, "CCW"))
                                print(f"  *** Angle→ CCW (settle {SETTLE_TIME_S:.1f}s)"
                                      f"  t={data.time:.2f}s ***")

            mujoco.mj_step(model, data)

            rpm_now  = float(data.qvel[vadr]) * 60.0 / (2.0 * math.pi)
            torq_now = float(data.actuator_force[aid_crank])
            fc_now   = _contact_force(model, data, bid_tablet)
            prev_rpm = rpm_now   # used by stall detection on next step

            t_log.append(data.time)
            rpm_log.append(rpm_now)
            torq_log.append(torq_now)
            f_log.append(fc_now.copy())

            if len(t_log) % 500 == 0:
                dir_s = "CCW" if direction == 1 else "CW"
                print(f"  {data.time:6.2f}s | {rpm_now:7.2f} | {torq_now:10.2f}    | "
                      f"{fc_now[1]:8.3f} | {data.ncon:4d}  [{dir_s}]")

            # Realtime plot update
            if len(t_log) % 20 == 0 and len(t_log) > 1:
                line_rpm.set_data(t_log,  rpm_log)
                line_torq.set_data(t_log, torq_log)
                line_fy.set_data(t_log,   [f[1] for f in f_log])
                for ax in (ax1, ax2):
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

    # Subplot 1: RPM + Torque
    ax_a.plot(t, rpm,  color="tab:purple", lw=1.8, label="Crank speed [RPM]")
    ax_a.plot(t, torq, color="tab:orange", lw=1.2, ls="--", alpha=0.85,
              label="Motor torque [N*m]")
    ax_a.axhline( target_rpm,      color="tab:purple", ls=":",  lw=1.0, alpha=0.5,
                  label=f"Target {target_rpm} RPM")
    ax_a.axhline(-target_rpm,      color="tab:purple", ls=":",  lw=1.0, alpha=0.5)
    ax_a.axhline( MOTOR_FORCELIM,  color="tab:orange", ls=":",  lw=0.8, alpha=0.4)
    ax_a.axhline(-MOTOR_FORCELIM,  color="tab:orange", ls=":",  lw=0.8, alpha=0.4)
    ax_a.axhline(0, color="gray", lw=0.5)
    _add_vlines(ax_a)
    ax_a.set_ylabel("RPM  /  N*m", fontsize=11)
    ax_a.set_title("Crank Speed & Motor Torque", fontsize=10)
    ax_a.legend(fontsize=9, loc="upper right"); ax_a.grid(True, alpha=0.3)

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
        print("Usage: python crusher_velocity_ctrl.py <path>.stl [--rpm 8] [--kv 60]")
        sys.exit(0)

    stl_path = os.path.abspath(stl_path)
    if not os.path.exists(stl_path):
        print(f"[ERROR] File not found: {stl_path}"); sys.exit(1)

    run(stl_path, density_kg_m3=args.density, kv=args.kv, target_rpm=args.rpm)

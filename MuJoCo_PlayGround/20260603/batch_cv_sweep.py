"""
batch_cv_sweep.py  — Headless batch runner: force profile vs. curvature (CV)

Runs crusher_velocity_ctrl simulation headlessly for each CV value,
saves an individual PNG per CV and a combined overlay PNG.

Usage:
    python batch_cv_sweep.py                          # R=4.0 AR=1.00 all CV
    python batch_cv_sweep.py --R 4.0 --AR 1.17       # specific R/AR group
    python batch_cv_sweep.py --density 1400 --rpm 8
    python batch_cv_sweep.py --csv_dir path/to/csvs  # re-plot from existing CSVs only

Output:
    MuJoCo_PlayGround/Sim_result/batch/
        cv_R4.0_AR1.00_CV0.08__result.png  (individual)
        ...
        cv_R4.0_AR1.00__overlay.png        (all CV overlaid)
        batch_summary.csv
"""

import os, sys, re, csv, math, glob, argparse
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use("Agg")          # headless — no display needed
import matplotlib.pyplot as plt
import matplotlib.cm as cm

# ── Paths ──────────────────────────────────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH    = os.path.normpath(os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml"))
MJCF_DIR     = os.path.dirname(MJCF_PATH)
STL_DIR      = os.path.normpath(os.path.join(_HERE, "..", "..", "tablets_stl", "stl"))
_SIM_RESULT  = os.path.normpath(os.path.join(_HERE, "..", "Sim_result"))
BATCH_DIR    = os.path.join(_SIM_RESULT, "batch")

# ── Sim constants (mirror crusher_velocity_ctrl.py) ────────────────────
PHASE1_STEPS   = 500
SIM_DURATION   = 40.0
MOTOR_DELAY    =  2.0
MOTOR_FORCELIM = 120.0
VEL_KV_DEFAULT =  60.0
CCW_REVS       =  1.0
CW_REVS        =  1.0

DENSITY_REF_SOFT = 900.0;   DENSITY_REF_HARD = 1800.0
SOLREF_TAU_SOFT  = 0.020;   SOLREF_TAU_HARD  = 0.002

PLACE_X_MM = -47.879
PLACE_Z_MM =  50.108
WALL_Y_MM  = 336.199

_s       = math.sqrt(2.0) / 2.0
TAB_QUAT = np.array([_s, 0.0, _s, 0.0])

try:
    _np_trapz = np.trapezoid
except AttributeError:
    _np_trapz = np.trapz


# ── Helpers ────────────────────────────────────────────────────────────
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
    import xml.etree.ElementTree as ET
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


# ── Headless simulation ────────────────────────────────────────────────
def run_headless(stl_path, density_kg_m3, kv, target_rpm):
    """Run simulation without viewer, return (t, rpm, torq, F) arrays."""
    target_vel = target_rpm * 2.0 * math.pi / 60.0

    fname = os.path.basename(stl_path)
    R_mm, AR, CV = _parse_params(fname)
    cd      = CV * 2 * R_mm
    th      = R_mm * 0.20 + 2 * cd
    half_th = th / 2.0

    model, (px, py, pz) = _build_model(stl_path, R_mm, half_th, density_kg_m3, kv, target_vel)
    data = mujoco.MjData(model)
    _dt  = float(model.opt.timestep)

    jid_crank  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,    "L3_Bevel_GearBox_1_L4_Shaft_1")
    aid_crank  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Motor1_crank")
    bid_tablet = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,     "tablet")
    eid_lock   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "lock_crank")
    mid_tab    = model.body_mocapid[bid_tablet]
    qadr       = model.jnt_qposadr[jid_crank]
    vadr       = model.jnt_dofadr[jid_crank]

    data.qpos[qadr]          = -math.pi / 2
    data.qvel[:]             = 0.0
    data.mocap_pos[mid_tab]  = [px, py, pz]
    data.mocap_quat[mid_tab] = TAB_QUAT
    data.ctrl[aid_crank]     = 0.0
    mujoco.mj_forward(model, data)

    # Phase 1
    for _ in range(PHASE1_STEPS):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    # Phase 2
    rev_angle   = 2.0 * math.pi
    p2_start    = data.time
    motor_on    = False
    motor_on_t  = None
    direction   = 1
    prev_ang    = float(data.qpos[qadr])
    acc_ccw = acc_cw = 0.0
    rev_events  = []

    t_log = []; f_log = []; rpm_log = []; torq_log = []

    total_steps = int(SIM_DURATION / _dt)
    for step in range(total_steps):
        if not motor_on and (data.time - p2_start) >= MOTOR_DELAY:
            data.eq_active[eid_lock] = 0
            direction = 1
            data.ctrl[aid_crank] = direction * target_vel
            motor_on = True; motor_on_t = data.time
            prev_ang = float(data.qpos[qadr])
            acc_ccw = acc_cw = 0.0

        if motor_on:
            cur_ang = float(data.qpos[qadr])
            delta   = ((cur_ang - prev_ang) + math.pi) % (2*math.pi) - math.pi
            prev_ang = cur_ang
            if direction == 1:
                acc_ccw += max(delta, 0.0)
                if acc_ccw >= CCW_REVS * rev_angle:
                    direction = -1
                    data.ctrl[aid_crank] = direction * target_vel
                    rev_events.append((data.time, "CW"))
                    acc_ccw = acc_cw = 0.0
            else:
                acc_cw += max(-delta, 0.0)
                if acc_cw >= CW_REVS * rev_angle:
                    direction = 1
                    data.ctrl[aid_crank] = direction * target_vel
                    rev_events.append((data.time, "CCW"))
                    acc_ccw = acc_cw = 0.0

        mujoco.mj_step(model, data)

        rpm_now  = float(data.qvel[vadr]) * 60.0 / (2.0 * math.pi)
        torq_now = float(data.actuator_force[aid_crank])
        fc_now   = _contact_force(model, data, bid_tablet)

        t_log.append(data.time)
        rpm_log.append(rpm_now)
        torq_log.append(torq_now)
        f_log.append(fc_now.copy())

        if step % 5000 == 0:
            print(f"    t={data.time:.1f}s  RPM={rpm_now:.2f}  Fy={fc_now[1]:.2f} N")

    t    = np.array(t_log)
    fc   = np.array(f_log)
    rpm  = np.array(rpm_log)
    torq = np.array(torq_log)
    return t, rpm, torq, fc, motor_on_t, rev_events


# ── Individual plot ────────────────────────────────────────────────────
def save_individual_plot(t, rpm, torq, fc, motor_on_t, rev_events,
                         R_mm, AR, CV, density, target_rpm, kv, out_path):
    fig, (ax_a, ax_b) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig.suptitle(
        f"Crusher {target_rpm} RPM  |  kv={kv:.0f}  lim={MOTOR_FORCELIM:.0f} N*m  |  "
        f"R={R_mm:.1f}  AR={AR:.2f}  CV={CV:.2f}  rho={density:.0f} kg/m3",
        fontsize=10, fontweight="bold")

    def _vlines(ax_):
        if motor_on_t:
            ax_.axvline(motor_on_t, color="tab:green", ls="--", lw=1.0,
                        label=f"Motor ON (t={motor_on_t:.1f}s)")
        for ev_t, ev_dir in rev_events:
            ax_.axvline(ev_t, color="tab:red", ls=":", lw=0.8)

    # Subplot 1: RPM + Torque
    ax_a.plot(t, rpm,  color="tab:purple", lw=1.5, label="Crank speed [RPM]")
    ax_a.plot(t, torq, color="tab:orange", lw=1.0, ls="--", alpha=0.85,
              label="Motor torque [N*m]")
    ax_a.axhline( target_rpm,     color="tab:purple", ls=":", lw=0.8, alpha=0.5,
                  label=f"Target {target_rpm} RPM")
    ax_a.axhline(-target_rpm,     color="tab:purple", ls=":", lw=0.8, alpha=0.5)
    ax_a.axhline( MOTOR_FORCELIM, color="tab:orange", ls=":", lw=0.6, alpha=0.4)
    ax_a.axhline(-MOTOR_FORCELIM, color="tab:orange", ls=":", lw=0.6, alpha=0.4)
    ax_a.axhline(0, color="gray", lw=0.5)
    _vlines(ax_a)
    ax_a.set_ylabel("RPM  /  N*m", fontsize=11)
    ax_a.set_title("Crank Speed & Motor Torque", fontsize=10)
    ax_a.legend(fontsize=9, loc="upper right"); ax_a.grid(True, alpha=0.3)

    # Subplot 2: Tablet F_Y
    F_Y_max = float(fc[:, 1].max())
    J_Y     = float(_np_trapz(fc[:, 1], t))
    ax_b.plot(t, fc[:, 1], color="tab:blue", lw=1.5, label="Tablet F_Y [N]")
    ax_b.fill_between(t, 0, fc[:, 1], where=(fc[:, 1] > 0),
                       alpha=0.12, color="tab:blue", label="compression")
    ax_b.fill_between(t, 0, fc[:, 1], where=(fc[:, 1] < 0),
                       alpha=0.12, color="tab:red",  label="tension")
    ax_b.axhline(0, color="k", lw=0.5)
    _vlines(ax_b)
    ax_b.set_ylabel("F_Y [N]", fontsize=11)
    ax_b.set_xlabel("Time [s]", fontsize=11)
    ax_b.set_title(f"Tablet Reaction Force  |  max={F_Y_max:.2f} N  J={J_Y:.4f} N*s",
                   fontsize=10)
    ax_b.legend(fontsize=9, loc="upper right"); ax_b.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [saved] {os.path.basename(out_path)}")


# ── Overlay plot (all CV on one figure) ───────────────────────────────
def save_overlay_plot(results, R_mm, AR, density, target_rpm, kv, out_path):
    """results: list of dict with keys cv, t, fc"""
    fig, ax = plt.subplots(figsize=(13, 6))
    cv_vals = sorted(set(r["cv"] for r in results))
    cmap    = cm.get_cmap("plasma", len(cv_vals))

    for i, r in enumerate(sorted(results, key=lambda x: x["cv"])):
        color = cmap(i / max(len(results) - 1, 1))
        label = f"CV={r['cv']:.2f}  (max={r['fy_max']:.1f} N)"
        ax.plot(r["t"], r["fc"][:, 1], color=color, lw=1.4, label=label)

    ax.axhline(0, color="k", lw=0.5)
    ax.set_xlabel("Time [s]", fontsize=12)
    ax.set_ylabel("F_Y [N]", fontsize=12)
    ax.set_title(
        f"Tablet Reaction Force vs. Curvature (CV)\n"
        f"R={R_mm:.1f}  AR={AR:.2f}  rho={density:.0f} kg/m3  "
        f"{target_rpm} RPM  kv={kv:.0f}  lim={MOTOR_FORCELIM:.0f} N*m",
        fontsize=11, fontweight="bold")
    ax.legend(fontsize=9, loc="upper right", ncol=2)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [saved overlay] {os.path.basename(out_path)}")


# ── Peak force bar chart ───────────────────────────────────────────────
def save_peak_bar(results, R_mm, AR, density, target_rpm, kv, out_path):
    results_s = sorted(results, key=lambda x: x["cv"])
    cvs    = [r["cv"]    for r in results_s]
    peaks  = [r["fy_max"] for r in results_s]
    impls  = [r["impulse"] for r in results_s]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    cmap  = cm.get_cmap("plasma", len(cvs))
    colors = [cmap(i / max(len(cvs) - 1, 1)) for i in range(len(cvs))]

    bars1 = ax1.bar([f"{c:.2f}" for c in cvs], peaks, color=colors, edgecolor="k", linewidth=0.5)
    ax1.set_xlabel("Curvature (CV)", fontsize=11)
    ax1.set_ylabel("Peak F_Y [N]", fontsize=11)
    ax1.set_title("Peak Reaction Force vs. CV", fontsize=11, fontweight="bold")
    ax1.bar_label(bars1, fmt="%.1f", fontsize=8, padding=2)
    ax1.grid(True, alpha=0.3, axis="y")

    bars2 = ax2.bar([f"{c:.2f}" for c in cvs], impls, color=colors, edgecolor="k", linewidth=0.5)
    ax2.set_xlabel("Curvature (CV)", fontsize=11)
    ax2.set_ylabel("Impulse J_Y [N*s]", fontsize=11)
    ax2.set_title("Total Impulse vs. CV", fontsize=11, fontweight="bold")
    ax2.bar_label(bars2, fmt="%.3f", fontsize=8, padding=2)
    ax2.grid(True, alpha=0.3, axis="y")

    fig.suptitle(
        f"Force Summary: R={R_mm:.1f}  AR={AR:.2f}  rho={density:.0f} kg/m3  "
        f"{target_rpm} RPM",
        fontsize=11, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [saved bar]     {os.path.basename(out_path)}")


# ── Load from CSV (re-plot mode) ───────────────────────────────────────
def load_csv(csv_path):
    """Return dict with t, rpm, torq, fc arrays parsed from saved CSV."""
    t_list = []; f_list = []; rpm_list = []; torq_list = []
    R_mm = AR = CV = density = None
    with open(csv_path, newline="", encoding="utf-8") as fh:
        reader = csv.reader(fh)
        for row in reader:
            if not row: continue
            if row[0].startswith("#"):
                # parse metadata rows
                if "R_mm" in row:
                    try: R_mm = float(row[row.index("R_mm") + 1])
                    except Exception: pass
                if "AR" in row:
                    try: AR = float(row[row.index("AR") + 1])
                    except Exception: pass
                if "CV" in row:
                    try: CV = float(row[row.index("CV") + 1])
                    except Exception: pass
                if "density_kg_m3" in row:
                    try: density = float(row[1])
                    except Exception: pass
                continue
            if row[0] == "Time_s": continue
            try:
                t_list.append(float(row[0]))
                f_list.append([float(row[1]), float(row[2]), float(row[3])])
                rpm_list.append(float(row[4]))
                torq_list.append(float(row[5]))
            except (ValueError, IndexError):
                continue

    if not t_list:
        return None
    return {
        "R_mm": R_mm, "AR": AR, "cv": CV, "density": density,
        "t":    np.array(t_list),
        "fc":   np.array(f_list),
        "rpm":  np.array(rpm_list),
        "torq": np.array(torq_list),
        "fy_max":  float(np.array(f_list)[:, 1].max()),
        "impulse": float(_np_trapz(np.array(f_list)[:, 1], np.array(t_list))),
    }


# ── Main ───────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Batch CV sweep: force profile vs. curvature")
    parser.add_argument("--R",        type=float, default=4.0,   help="Tablet radius [mm] (default 4.0)")
    parser.add_argument("--AR",       type=float, default=1.00,  help="Aspect ratio (default 1.00)")
    parser.add_argument("--density",  type=float, default=1200.0)
    parser.add_argument("--rpm",      type=float, default=8.0)
    parser.add_argument("--kv",       type=float, default=60.0)
    parser.add_argument("--csv_dir",  type=str,   default=None,
                        help="Re-plot from existing CSVs in this directory (skip simulation)")
    parser.add_argument("--stl_dir",  type=str,   default=None,
                        help="Override STL directory")
    args = parser.parse_args()

    os.makedirs(BATCH_DIR, exist_ok=True)

    # ── Re-plot from existing CSVs ──────────────────────────────────────
    if args.csv_dir:
        print(f"\n[Re-plot mode] Loading CSVs from: {args.csv_dir}")
        csv_files = sorted(glob.glob(os.path.join(args.csv_dir, "*.csv")))
        if not csv_files:
            print("[ERROR] No CSV files found."); sys.exit(1)

        results = []
        for cp in csv_files:
            r = load_csv(cp)
            if r is None:
                print(f"  [skip] {os.path.basename(cp)} (parse failed)"); continue
            results.append(r)
            print(f"  Loaded CV={r['cv']:.2f}  fy_max={r['fy_max']:.2f} N")

        if not results:
            print("[ERROR] No valid data loaded."); sys.exit(1)

        R_mm    = results[0]["R_mm"]   or args.R
        AR      = results[0]["AR"]     or args.AR
        density = results[0]["density"] or args.density

        for r in results:
            out_png = os.path.join(BATCH_DIR,
                f"cv_R{R_mm:.1f}_AR{AR:.2f}_CV{r['cv']:.2f}__result.png")
            save_individual_plot(r["t"], r["rpm"], r["torq"], r["fc"],
                                 None, [],
                                 R_mm, AR, r["cv"], density,
                                 args.rpm, args.kv, out_png)

        overlay_png = os.path.join(BATCH_DIR, f"cv_R{R_mm:.1f}_AR{AR:.2f}__overlay.png")
        save_overlay_plot(results, R_mm, AR, density, args.rpm, args.kv, overlay_png)

        bar_png = os.path.join(BATCH_DIR, f"cv_R{R_mm:.1f}_AR{AR:.2f}__peak_bar.png")
        save_peak_bar(results, R_mm, AR, density, args.rpm, args.kv, bar_png)
        print("\n[Done] Re-plot complete.")
        return

    # ── Run simulations ─────────────────────────────────────────────────
    global mujoco
    try:
        import mujoco
    except ImportError as e:
        print(f"[ERROR] Cannot import mujoco: {e}")
        print("  Run from conda env:  conda activate isaac_sim")
        sys.exit(1)

    stl_dir = args.stl_dir or STL_DIR
    pattern = os.path.join(stl_dir, f"tablet_R{args.R:.1f}_AR{args.AR:.2f}_CV*.stl")
    stl_files = sorted(glob.glob(pattern))
    if not stl_files:
        print(f"[ERROR] No STL files found: {pattern}"); sys.exit(1)

    print(f"\n{'='*64}")
    print(f"  Batch CV Sweep — R={args.R:.1f}  AR={args.AR:.2f}  "
          f"rho={args.density:.0f}  {args.rpm} RPM")
    print(f"  Found {len(stl_files)} tablet(s)")
    print(f"{'='*64}\n")

    results     = []
    summary_rows = []
    csv_dir_out  = os.path.join(_SIM_RESULT, "csv")
    os.makedirs(csv_dir_out, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    for i, stl_path in enumerate(stl_files):
        fname = os.path.basename(stl_path)
        R_mm, AR, CV = _parse_params(fname)
        print(f"\n[{i+1}/{len(stl_files)}] CV={CV:.2f}  ({fname})")

        t, rpm, torq, fc, motor_on_t, rev_events = run_headless(
            stl_path, args.density, args.kv, args.rpm)

        fy_max  = float(fc[:, 1].max())
        fy_min  = float(fc[:, 1].min())
        impulse = float(_np_trapz(fc[:, 1], t))

        print(f"  CV={CV:.2f}  fy_max={fy_max:.2f} N  impulse={impulse:.4f} N*s")

        # Save CSV
        stem     = f"batch_{os.path.splitext(fname)[0]}__{ts}"
        csv_path = os.path.join(csv_dir_out, f"{stem}.csv")
        with open(csv_path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["# Batch CV Sweep"])
            w.writerow(["# STL", fname, "R_mm", R_mm, "AR", AR, "CV", CV])
            w.writerow(["# target_rpm", args.rpm, "kv", args.kv,
                        "forcelim", MOTOR_FORCELIM])
            w.writerow(["# density_kg_m3", args.density])
            w.writerow(["# F_Y_max", f"{fy_max:.5f}", "F_Y_min", f"{fy_min:.5f}",
                        "Impulse_Ns", f"{impulse:.6f}"])
            w.writerow([])
            w.writerow(["Time_s", "F_X_N", "F_Y_N", "F_Z_N",
                        "Crank_RPM", "Motor_torque_Nm"])
            for j in range(len(t)):
                w.writerow([f"{t[j]:.5f}",
                            f"{fc[j,0]:.5f}", f"{fc[j,1]:.5f}", f"{fc[j,2]:.5f}",
                            f"{rpm[j]:.4f}", f"{torq[j]:.4f}"])

        # Save individual plot
        out_png = os.path.join(BATCH_DIR,
            f"cv_R{R_mm:.1f}_AR{AR:.2f}_CV{CV:.2f}__result.png")
        save_individual_plot(t, rpm, torq, fc, motor_on_t, rev_events,
                             R_mm, AR, CV, args.density,
                             args.rpm, args.kv, out_png)

        results.append({
            "cv": CV, "t": t, "fc": fc, "rpm": rpm, "torq": torq,
            "fy_max": fy_max, "impulse": impulse,
        })
        summary_rows.append({
            "STL": fname, "R_mm": R_mm, "AR": AR, "CV": CV,
            "density": args.density, "fy_max": fy_max,
            "fy_min": fy_min, "impulse": impulse,
            "rpm_target": args.rpm, "kv": args.kv,
        })

    # Overlay + bar chart
    overlay_png = os.path.join(BATCH_DIR, f"cv_R{args.R:.1f}_AR{args.AR:.2f}__overlay.png")
    save_overlay_plot(results, args.R, args.AR, args.density, args.rpm, args.kv, overlay_png)

    bar_png = os.path.join(BATCH_DIR, f"cv_R{args.R:.1f}_AR{args.AR:.2f}__peak_bar.png")
    save_peak_bar(results, args.R, args.AR, args.density, args.rpm, args.kv, bar_png)

    # Summary CSV
    sum_csv = os.path.join(BATCH_DIR, f"batch_summary_R{args.R:.1f}_AR{args.AR:.2f}__{ts}.csv")
    with open(sum_csv, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(summary_rows[0].keys()))
        w.writeheader(); w.writerows(summary_rows)
    print(f"\n  [saved summary] {os.path.basename(sum_csv)}")

    print(f"\n{'='*64}")
    print(f"  All done.  Output: {BATCH_DIR}")
    print(f"{'='*64}")

    # Print table
    print(f"\n  {'CV':>6} | {'F_Y_max [N]':>12} | {'Impulse [N*s]':>14}")
    print("  " + "-" * 40)
    for r in sorted(results, key=lambda x: x["cv"]):
        print(f"  {r['cv']:6.2f} | {r['fy_max']:12.3f} | {r['impulse']:14.5f}")


if __name__ == "__main__":
    main()

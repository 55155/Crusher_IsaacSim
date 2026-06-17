"""
crusher_velocity_ctrl_headless_genesis.py
Genesis 1.1.2 (CPU backend) 포트.

원본: ../20260603/crusher_velocity_ctrl_viewer.py  (MuJoCo + viewer)
차이점:
  - 백엔드: MuJoCo  →  Genesis (CPU, headless)
  - viewer X, realtime plot X  → 헤드리스 시뮬 후 최종 plot 저장
  - lock_crank equality (joint polycoef) 제거 → qpos 직접 초기화
  - tablet: mocap body → kinematic morph (Mesh, fixed=True)

Usage:
  python crusher_velocity_ctrl_headless_genesis.py [tablet.stl] [--rpm 8] [--density 1200]
"""

import os, sys, re, csv, math, argparse, time, xml.etree.ElementTree as ET
from datetime import datetime

import numpy as np
try:
    _np_trapz = np.trapezoid
except AttributeError:
    _np_trapz = np.trapz

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import genesis as gs
import torch

# ── Paths ────────────────────────────────────────────────────────────
_HERE     = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.normpath(os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml"))
MJCF_DIR  = os.path.dirname(MJCF_PATH)
STL_DIR   = os.path.normpath(os.path.join(_HERE, "..", "..", "tablets_stl", "stl"))
_SIM_RESULT = os.path.normpath(os.path.join(_HERE, "..", "Sim_result"))
CSV_DIR   = os.path.join(_SIM_RESULT, "csv")
PLOT_DIR  = os.path.join(_SIM_RESULT, "plot")

# ── Placement (원본과 동일) ──────────────────────────────────────────
PLACE_X_MM = -47.879
PLACE_Z_MM =  50.108
WALL_Y_MM  = 336.199

# ── Simulation params ────────────────────────────────────────────────
SIM_DT         = 0.001
PHASE1_STEPS   = 500
SIM_DURATION   = 20.0
MOTOR_DELAY    =  2.0
TARGET_RPM     =  8.0

# ── Motor ────────────────────────────────────────────────────────────
TARGET_VEL_RAD_S = TARGET_RPM * 2.0 * math.pi / 60.0
CRANK_JOINT_NAME = "L3_Bevel_GearBox_1_L4_Shaft_1"
CRANK_INIT_RAD   = -math.pi / 2.0

# ── Tablet ───────────────────────────────────────────────────────────
DENSITY_DEFAULT = 1200.0
_s = math.sqrt(2.0) / 2.0
TAB_QUAT = (_s, 0.0, _s, 0.0)   # (w, x, y, z) — MuJoCo 컨벤션


# ─────────────────────────────────────────────────────────────────────
def _parse_stl_params(fname):
    m = re.search(r"R([\d.]+)_AR([\d.]+)_CV([\d.]+)",
                  os.path.splitext(os.path.basename(fname))[0])
    return (float(m.group(1)), float(m.group(2)), float(m.group(3))) if m else (None, None, None)


def _prepare_mjcf(mjcf_path, out_path):
    """
    원본 MJCF 에서 Genesis가 처리 못하는 요소 제거 / 정규화:
      - <equality><joint polycoef=... name=lock_crank>  제거
        (Genesis 미지원 — 대신 qpos 초기화로 처리)
      - <keyframe>  제거
      - compiler/meshdir 절대경로화
    """
    tree = ET.parse(mjcf_path)
    root = tree.getroot()

    # meshdir 절대경로
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("meshdir", MJCF_DIR)

    # keyframe 제거
    for kf in list(root.findall("keyframe")):
        root.remove(kf)

    # equality 내부 lock_crank (joint polycoef) 제거 — weld 는 유지
    # weld 의 solref/solimp 는 MuJoCo negative direct mode → Genesis 미지원, 제거
    eq = root.find("equality")
    if eq is not None:
        for child in list(eq):
            if child.tag == "joint" and child.get("name") == "lock_crank":
                eq.remove(child)
            elif child.tag == "weld":
                child.attrib.pop("solref", None)
                child.attrib.pop("solimp", None)

    # actuator 정규화: velocity actuator 로 교체
    act = root.find("actuator")
    if act is not None:
        for elem in list(act):
            if elem.get("name") == "Motor1_crank":
                act.remove(elem)
        # forcelim 동일하게 ±12.5 N·m
        ET.SubElement(act, "velocity", {
            "name":       "Motor1_crank",
            "joint":      CRANK_JOINT_NAME,
            "kv":         "14.9",
            "gear":       "1",
            "forcerange": "-12.5 12.5",
            "ctrlrange":  f"{-TARGET_VEL_RAD_S*2:.4f} {TARGET_VEL_RAD_S*2:.4f}",
        })

    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return out_path


def _build_scene(stl_path, R_mm, half_th, density_kg_m3):
    """Genesis Scene + Crusher MJCF + Tablet mesh."""
    # 1. MJCF 전처리
    tmp_mjcf = os.path.join(MJCF_DIR, "_genesis_tmp.xml")
    _prepare_mjcf(MJCF_PATH, tmp_mjcf)

    # 2. Scene
    scene = gs.Scene(
        sim_options    = gs.options.SimOptions(dt=SIM_DT, substeps=1),
        rigid_options  = gs.options.RigidOptions(gravity=(0,0,-9.81)),
        show_viewer    = False,
        show_FPS       = False,
    )

    # 3. Crusher 본체
    crusher = scene.add_entity(
        gs.morphs.MJCF(file=tmp_mjcf),
    )

    # 4. Tablet — kinematic (fixed) STL morph
    pos_x_m = PLACE_X_MM * 1e-3
    pos_z_m = PLACE_Z_MM * 1e-3
    pos_y_m = (WALL_Y_MM - half_th) * 1e-3

    tablet = scene.add_entity(
        morph = gs.morphs.Mesh(
            file   = stl_path,
            scale  = 0.001,                  # mm → m
            pos    = (pos_x_m, pos_y_m, pos_z_m),
            quat   = TAB_QUAT,
            fixed  = True,                   # kinematic — 외력에 안 움직임
        ),
        material = gs.materials.Rigid(
            rho            = density_kg_m3,
            friction       = 0.5,
        ),
    )

    return scene, crusher, tablet, (pos_x_m, pos_y_m, pos_z_m)


def _get_link_contact_force_y(scene, entity):
    """tablet entity 에 작용하는 net contact force 의 Y 성분 [N]."""
    try:
        f = entity.get_links_net_contact_force()   # shape (n_links,3) 또는 (1,n_links,3)
        if hasattr(f, "cpu"):
            f = f.cpu().numpy()
        f = np.asarray(f).reshape(-1, 3)
        return float(f[:, 1].sum())
    except Exception:
        return 0.0


# ─────────────────────────────────────────────────────────────────────
def run(stl_path, density_kg_m3=DENSITY_DEFAULT, target_rpm=TARGET_RPM):
    target_vel = target_rpm * 2.0 * math.pi / 60.0

    print("=" * 64)
    print(f"  Crusher [Genesis CPU — {target_rpm} RPM, headless]")
    print("=" * 64)

    fname = os.path.basename(stl_path)
    R_mm, AR, CV = _parse_stl_params(fname)
    if R_mm is None:
        print("[ERROR] Filename parse failed"); sys.exit(1)

    cd      = CV * 2 * R_mm
    th      = R_mm * 0.20 + 2 * cd
    half_th = th / 2.0

    print(f"  STL    : {fname}")
    print(f"  R={R_mm:.1f} AR={AR:.2f} CV={CV:.2f}  thickness={th:.2f}mm")
    print(f"  Target : {target_rpm} RPM = {target_vel:.4f} rad/s")
    print(f"  Backend: Genesis CPU\n")

    os.makedirs(CSV_DIR, exist_ok=True)
    os.makedirs(PLOT_DIR, exist_ok=True)
    ts          = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_stem = f"genesis_velctrl_{os.path.splitext(fname)[0]}__{ts}"

    # ── Genesis init ────────────────────────────────────────────────
    gs.init(backend=gs.cpu, logging_level="warning")

    scene, crusher, tablet, (px, py, pz) = _build_scene(stl_path, R_mm, half_th, density_kg_m3)

    print("  Building scene ...")
    t0 = time.time()
    scene.build()
    print(f"  Build OK ({time.time()-t0:.1f}s)\n")

    # ── DOF / actuator 인덱스 ────────────────────────────────────────
    crank_dof = None
    for j in crusher.joints:
        if j.name == CRANK_JOINT_NAME:
            crank_dof = j.dofs_idx_local
            break
    if crank_dof is None:
        print(f"[ERROR] crank joint '{CRANK_JOINT_NAME}' not found"); sys.exit(1)

    # ── Phase 1: crank 초기 각도 설정 + settle ──────────────────────
    print(f"[Phase 1] Initialize crank @ {math.degrees(CRANK_INIT_RAD):.1f}° + {PHASE1_STEPS} settle steps")
    qpos_init = torch.zeros(crusher.n_dofs)
    qpos_init[crank_dof[0]] = CRANK_INIT_RAD
    crusher.set_dofs_position(qpos_init)

    for _ in range(PHASE1_STEPS):
        scene.step()
    print(f"  Done  crank={math.degrees(crusher.get_dofs_position(crank_dof).cpu().numpy()[0]):.1f}°\n")

    # ── Phase 2: motor on after delay, run SIM_DURATION ─────────────
    p2_start   = scene.cur_t
    motor_on   = False
    motor_on_t = None

    t_log = []; rpm_log = []; ang_log = []; fy_log = []

    print(f"[Phase 2] Motor ON at t+{MOTOR_DELAY}s  (max {SIM_DURATION}s)")
    print(f"  {'Time':>6s} | {'RPM':>7s} | {'Crank°':>8s} | {'F_Y':>8s} N")
    print("  " + "-" * 44)
    print_every = 1000
    n_steps = int(SIM_DURATION / SIM_DT)
    sim_t0 = time.time()

    for k in range(n_steps):
        t_cur = scene.cur_t

        if not motor_on and (t_cur - p2_start) >= MOTOR_DELAY:
            motor_on   = True
            motor_on_t = t_cur
            print(f"  *** Motor ON  t={t_cur:.2f}s  target={target_vel:.4f} rad/s ***")

        if motor_on:
            crusher.control_dofs_velocity(
                torch.tensor([target_vel], dtype=torch.float32), crank_dof
            )

        scene.step()

        # 기록
        ang_rad = float(crusher.get_dofs_position(crank_dof).cpu().numpy()[0])
        vel_rad = float(crusher.get_dofs_velocity(crank_dof).cpu().numpy()[0])
        rpm_now = vel_rad * 60.0 / (2.0 * math.pi)
        ang_deg = math.degrees(ang_rad)
        fy      = _get_link_contact_force_y(scene, tablet)

        t_log.append(t_cur)
        rpm_log.append(rpm_now)
        ang_log.append(ang_deg)
        fy_log.append(fy)

        if (k + 1) % print_every == 0:
            print(f"  {t_cur:6.2f}s | {rpm_now:7.2f} | {ang_deg:8.1f} | {fy:8.3f}")

    print(f"\n  Sim wall-time: {time.time()-sim_t0:.1f}s")

    # ── 결과 정리 ───────────────────────────────────────────────────
    t   = np.array(t_log)
    rpm = np.array(rpm_log)
    ang = np.array(ang_log)
    fy  = np.array(fy_log)

    F_Y_max = float(fy.max())
    F_Y_min = float(fy.min())
    J_Y     = float(_np_trapz(fy, t))

    print(f"\n  {'='*55}")
    print(f"  Steps      : {len(t)}  ({t[-1]:.2f} s)")
    print(f"  RPM range  : {rpm.min():.2f} ~ {rpm.max():.2f}  (target {target_rpm})")
    print(f"  F_Y        : {F_Y_min:.3f} ~ {F_Y_max:.3f} N")
    print(f"  Impulse    : {J_Y:.5f} N*s")
    print(f"  {'='*55}\n")

    # ── 최종 plot ───────────────────────────────────────────────────
    fig, (ax_a, ax_b) = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    fig.suptitle(
        f"Genesis CPU | {target_rpm} RPM  |  "
        f"R={R_mm:.1f} AR={AR:.2f} CV={CV:.2f}  ρ={density_kg_m3:.0f} kg/m³",
        fontsize=10, fontweight="bold")

    ax_a.plot(t, rpm, color="tab:purple", lw=1.5, label="Crank RPM")
    ax_a.axhline(target_rpm, color="tab:purple", ls=":", lw=1.0, alpha=0.5,
                 label=f"Target {target_rpm} RPM")
    if motor_on_t:
        ax_a.axvline(motor_on_t, color="tab:green", ls="--", lw=1.0,
                     label=f"Motor ON (t={motor_on_t:.1f}s)")
    ax_a.set_ylabel("RPM"); ax_a.grid(True, alpha=0.3); ax_a.legend(fontsize=9)
    ax_ab = ax_a.twinx()
    ax_ab.plot(t, ang, color="tab:green", lw=1.0, ls="-.", alpha=0.7, label="Crank°")
    ax_ab.set_ylabel("Crank angle [°]", color="tab:green")
    ax_ab.tick_params(axis='y', labelcolor='tab:green')

    ax_b.plot(t, fy, color="tab:blue", lw=1.5, label="Tablet F_Y")
    ax_b.fill_between(t, 0, fy, where=(fy>0), alpha=0.15, color="tab:blue")
    ax_b.fill_between(t, 0, fy, where=(fy<0), alpha=0.15, color="tab:red")
    ax_b.axhline(0, color="k", lw=0.5)
    if motor_on_t:
        ax_b.axvline(motor_on_t, color="tab:green", ls="--", lw=1.0)
    ax_b.set_ylabel("F_Y [N]"); ax_b.set_xlabel("Time [s]")
    ax_b.set_title(f"max={F_Y_max:.2f} N  J={J_Y:.4f} N·s", fontsize=10)
    ax_b.grid(True, alpha=0.3); ax_b.legend(fontsize=9)

    fig.tight_layout()

    p_out = os.path.join(PLOT_DIR, f"{result_stem}__result.png")
    csv_p = os.path.join(CSV_DIR,  f"{result_stem}.csv")
    fig.savefig(p_out, dpi=150, bbox_inches="tight")
    plt.close(fig)

    with open(csv_p, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["# Genesis CPU velocity control"])
        w.writerow(["# STL", fname, "R_mm", R_mm, "AR", AR, "CV", CV])
        w.writerow(["# target_rpm", target_rpm, "density_kg_m3", density_kg_m3])
        w.writerow(["# F_Y_max", f"{F_Y_max:.5f}", "F_Y_min", f"{F_Y_min:.5f}",
                    "Impulse_Ns", f"{J_Y:.6f}"])
        w.writerow([])
        w.writerow(["Time_s", "Crank_deg", "F_Y_N", "Crank_RPM"])
        for i in range(len(t_log)):
            w.writerow([f"{t_log[i]:.5f}", f"{ang_log[i]:.3f}",
                        f"{fy_log[i]:.5f}", f"{rpm_log[i]:.4f}"])

    print(f"  Saved: {os.path.basename(p_out)}")
    print(f"  Saved: {os.path.basename(csv_p)}")


# ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Crusher Genesis CPU headless")
    parser.add_argument("stl",       nargs="?", default=None)
    parser.add_argument("--rpm",     type=float, default=TARGET_RPM)
    parser.add_argument("--density", type=float, default=DENSITY_DEFAULT)
    args = parser.parse_args()

    stl_path = args.stl
    if stl_path is None:
        # 기본 STL 자동 선택
        if os.path.isdir(STL_DIR):
            cands = sorted([f for f in os.listdir(STL_DIR) if f.endswith(".stl")])
            if cands:
                stl_path = os.path.join(STL_DIR, cands[len(cands)//2])
                print(f"[INFO] Auto-selected STL: {os.path.basename(stl_path)}")
    if not stl_path or not os.path.exists(stl_path):
        print("Usage: python crusher_velocity_ctrl_headless_genesis.py <path>.stl")
        sys.exit(1)

    run(os.path.abspath(stl_path),
        density_kg_m3=args.density,
        target_rpm=args.rpm)

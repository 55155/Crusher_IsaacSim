"""
crusher_tablet_sim.py  [v3 — 2-Phase, 알약 고정]
Crusher + Tablet 통합 시뮬레이션

▶ Phase 1  (PHASE1_STEPS 스텝, 뷰어 없음, 빠름)
    크랭크 90° 초기화 + Tablet 공중 고정 → 메커니즘 안정화

▶ Phase 2  (뷰어 오픈)
    Tablet 위치 고정 유지 → 접촉 반력 측정

▶ 배치 좌표 (MuJoCo world frame)
    PLACE_X_MM = -47.879  →  MuJoCo X
    PLACE_Z_MM =  50.108  →  MuJoCo Z
    WALL_Y_MM  = 336.199  →  MuJoCo Y  (충돌판 벽)
    Tablet Y   = (WALL_Y_MM - R_mm) / 1000   ← 단반경만큼 벽에서 이격

▶ 실행
    conda activate isaac_sim
    python crusher_tablet_sim.py [tablet.stl]
"""

import os
import sys
import re
import argparse
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer

# ── 경로 ─────────────────────────────────────────────────────────────
_HERE     = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.normpath(
    os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml"))
MJCF_DIR  = os.path.dirname(MJCF_PATH)
STL_DIR   = os.path.normpath(
    os.path.join(_HERE, "..", "..", "tablets_stl", "stl"))

# ── 배치 좌표 (mm, MuJoCo world frame) ──────────────────────────────
PLACE_X_MM = -47.879
PLACE_Z_MM =  50.108
WALL_Y_MM  = 336.199   # impact plate 벽 위치

# ── Phase 1 스텝 수 (뷰어 없이 안정화) ──────────────────────────────
PHASE1_STEPS = 500

# ── Phase 2 시뮬레이션 파라미터 ──────────────────────────────────────
SIM_DURATION = 30.0   # 측정 시간 [s]
MOTOR_CTRL   = 0.5    # Motor1_crank 제어 입력 [N·m]


# ─────────────────────────────────────────────────────────────────────
def _parse_params(fname: str):
    """파일명에서 R, AR, CV 파라미터 추출."""
    stem = os.path.splitext(os.path.basename(fname))[0]
    m = re.search(r"R([\d.]+)_AR([\d.]+)_CV([\d.]+)", stem)
    if not m:
        return None, None, None
    return float(m.group(1)), float(m.group(2)), float(m.group(3))


# ─────────────────────────────────────────────────────────────────────
def _build_model(stl_path: str, R_mm: float):
    """
    Crusher XML + Tablet STL → MjModel (메모리 내 조합).

    수정 사항:
      ① meshdir 설정   → Crusher STL 절대경로 참조
      ② keyframe 제거  → nq 불일치(8 vs 15) 방지
      ③ tablet body    → freejoint + force site + geom
      ④ sensor 추가    → force / torque 측정
    """
    # ── 배치 위치 계산 ────────────────────────────────────────────────
    pos_x = PLACE_X_MM * 1e-3                  # MuJoCo X [m]
    pos_z = PLACE_Z_MM * 1e-3                  # MuJoCo Z [m]
    pos_y = WALL_Y_MM * 1e-3                   # MuJoCo Y [m]  (중심 = 벽면)

    print(f"  배치 [mm] : X={PLACE_X_MM:.3f}  "
          f"Y={WALL_Y_MM:.3f}  Z={PLACE_Z_MM:.3f}")
    print(f"  배치 [m]  : X={pos_x:.5f}  Y={pos_y:.5f}  Z={pos_z:.5f}")
    print(f"  ( 알약 중심 = WALL_Y={WALL_Y_MM:.1f}mm, 벽에 완전 접촉 )")

    # ── Crusher XML 파싱 ─────────────────────────────────────────────
    tree = ET.parse(MJCF_PATH)
    root = tree.getroot()

    # ① meshdir → Crusher MJCF 디렉토리 절대경로
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", MJCF_DIR)

    # ② keyframe 제거 (nq=8로 정의되어 있으나 tablet freejoint 추가 후 nq=15)
    for kf in root.findall("keyframe"):
        root.remove(kf)

    # ③-a tablet mesh + material asset
    asset = root.find("asset")
    ET.SubElement(asset, "mesh", {
        "name":  "tablet_mesh",
        "file":  "tablet.stl",          # assets dict 에서 바이트로 공급
        "scale": ".001 .001 .001",
    })
    ET.SubElement(asset, "material", {
        "name":      "tablet_mat",
        "rgba":      ".85 .80 .72 1",
        "specular":  ".4",
        "shininess": ".3",
    })

    # ③-b tablet body  (freejoint 포함)
    #   quat = 90° around world-X  →  local-Z(두께) → world-Y(충돌 방향)
    worldbody = root.find("worldbody")
    tab = ET.SubElement(worldbody, "body", {
        "name": "tablet",
        "pos":  f"{pos_x:.6f} {pos_y:.6f} {pos_z:.6f}",
        "quat": "0.7071068 0.7071068 0 0",
    })
    ET.SubElement(tab, "freejoint", {"name": "tablet_free"})
    ET.SubElement(tab, "site", {
        "name": "tablet_force_site",
        "pos":  "0 0 0",
        "size": "0.005",
    })
    ET.SubElement(tab, "geom", {
        "name":     "tablet_geom",
        "type":     "mesh",
        "mesh":     "tablet_mesh",
        "material": "tablet_mat",
        "density":  "1200",
        "condim":   "4",
        "friction": ".5 .02 .01",
    })

    # ④ force / torque 센서
    sensor_sec = ET.SubElement(root, "sensor")
    ET.SubElement(sensor_sec, "force",
                  {"name": "tablet_force",  "site": "tablet_force_site"})
    ET.SubElement(sensor_sec, "torque",
                  {"name": "tablet_torque", "site": "tablet_force_site"})

    # ── 조합 → MjModel ───────────────────────────────────────────────
    xml_str   = ET.tostring(root, encoding="unicode")
    stl_bytes = open(stl_path, "rb").read()
    model     = mujoco.MjModel.from_xml_string(
        xml_str, assets={"tablet.stl": stl_bytes})
    return model, (pos_x, pos_y, pos_z)


# ─────────────────────────────────────────────────────────────────────
def run(stl_path: str):
    print("=" * 62)
    print("  Crusher + Tablet  2-Phase 통합 시뮬레이션  [알약 고정]")
    print("=" * 62)

    fname = os.path.basename(stl_path)
    R_mm, AR, CV = _parse_params(fname)
    if R_mm is None:
        print(f"[ERROR] 파일명 파싱 실패: {fname}")
        sys.exit(1)

    cd = CV * 2 * R_mm
    th = R_mm * 0.20 + 2 * cd
    print(f"  STL  : {fname}")
    print(f"  R={R_mm:.1f}mm  AR={AR:.2f}  CV={CV:.2f}  두께≈{th:.2f}mm")
    print()

    # ── 모델 로드 ────────────────────────────────────────────────────
    model, (px, py, pz) = _build_model(stl_path, R_mm)
    data = mujoco.MjData(model)

    # ── ID 조회 ──────────────────────────────────────────────────────
    crank_jid  = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "L3_Bevel_GearBox_1_L4_Shaft_1")
    tab_jid    = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "tablet_free")
    act_crank  = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Motor1_crank")
    b_tablet   = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "tablet")
    b_slider   = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "L8_Link3_Shaft_1")
    s_id       = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_SENSOR, "tablet_force")

    crank_qadr = model.jnt_qposadr[crank_jid]
    tab_qadr   = model.jnt_qposadr[tab_jid]
    tab_vadr   = model.jnt_dofadr[tab_jid]
    s_adr      = model.sensor_adr[s_id]

    print(f"\n  nq={model.nq}  |  "
          f"crank qpos[{crank_qadr}]  "
          f"tablet qpos[{tab_qadr}:{tab_qadr+7}]")

    # ── ❶ 초기 상태 설정 (qpos 직접 지정) ──────────────────────────
    #   크랭크 90°, tablet 목표 위치, 속도 0
    data.qpos[crank_qadr]          = np.pi / 2    # 90°
    data.qpos[tab_qadr:tab_qadr+3] = [px, py, pz]
    data.qpos[tab_qadr+3]          = 1.0          # qw
    data.qpos[tab_qadr+4:tab_qadr+7] = 0.0        # qx qy qz
    data.qvel[:]                   = 0.0
    mujoco.mj_forward(model, data)

    # ── ❷ Phase 1: 메커니즘 안정화, tablet 공중 고정 ────────────────
    print(f"\n◆ Phase 1: {PHASE1_STEPS} 스텝 안정화 (뷰어 없음)")
    print(f"  tablet 고정 위치: ({px:.4f}, {py:.4f}, {pz:.4f}) m")

    tab_pin = np.array([px, py, pz, 1.0, 0.0, 0.0, 0.0])

    for step in range(PHASE1_STEPS):
        mujoco.mj_step(model, data)
        # tablet kinematic hold: 매 스텝 qpos·qvel 덮어쓰기
        data.qpos[tab_qadr:tab_qadr+7] = tab_pin
        data.qvel[tab_vadr:tab_vadr+6] = 0.0

    mujoco.mj_forward(model, data)
    crank_deg = np.degrees(data.qpos[crank_qadr])
    print(f"  ✔ Phase 1 완료  crank={crank_deg:.1f}°  "
          f"sim_time={data.time:.3f}s")

    # ── ❸ Phase 2: tablet 고정 유지 + 뷰어 + 측정 ───────────────────
    print(f"\n◆ Phase 2: tablet 고정 유지 → 뷰어 오픈")
    print(f"  Motor1_crank ctrl = {MOTOR_CTRL} N·m")
    print(f"  측정 시간 = {SIM_DURATION} s\n")

    data.ctrl[act_crank] = MOTOR_CTRL   # 모터 ON

    t_log    = []
    f_ext    = []
    f_sens   = []
    slider_y = []
    tablet_y = []
    gap_log  = []
    first_contact_t = None

    print(f"  {'Time':>6s} | {'Slider_Y':>9s} mm | {'Tablet_Y':>9s} mm | "
          f"{'Gap':>7s} mm | {'F_Y':>8s} N")
    print("  " + "-" * 66)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT]      = False
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = False
        viewer.opt.frame                                      = mujoco.mjtFrame.mjFRAME_NONE
        viewer.opt.geomgroup[3]                               = False
        for sg in range(5):
            viewer.opt.sitegroup[sg] = False

        while viewer.is_running() and data.time < SIM_DURATION:
            mujoco.mj_step(model, data)

            # 알약 위치·속도 고정
            data.qpos[tab_qadr:tab_qadr+7] = tab_pin
            data.qvel[tab_vadr:tab_vadr+6] = 0.0

            sy     = float(data.xpos[b_slider, 1])
            ty_now = float(data.xpos[b_tablet, 1])
            gap_mm = (ty_now - sy) * 1e3

            fe_now = data.cfrc_ext[b_tablet, 3:6].copy()
            fs_now = data.sensordata[s_adr:s_adr+3].copy()

            t_log.append(data.time)
            f_ext.append(fe_now)
            f_sens.append(fs_now)
            slider_y.append(sy)
            tablet_y.append(ty_now)
            gap_log.append(gap_mm)

            # 첫 접촉 감지
            if first_contact_t is None and abs(fe_now[1]) > 0.1:
                first_contact_t = data.time
                print(f"  *** 첫 접촉: t={data.time:.3f}s  "
                      f"F_Y={fe_now[1]:.2f}N  gap={gap_mm:.2f}mm ***")

            # 500 스텝마다 콘솔 출력
            if len(t_log) % 500 == 0:
                print(f"  {data.time:6.2f}s | {sy*1e3:9.2f}    | "
                      f"{ty_now*1e3:9.2f}    | {gap_mm:7.2f}    | "
                      f"{fe_now[1]:8.3f}")

            viewer.sync()

    # ── 결과 집계 ─────────────────────────────────────────────────────
    if not t_log:
        print("[경고] 데이터 없음.")
        return

    t   = np.array(t_log)
    fe  = np.array(f_ext)
    fs  = np.array(f_sens)
    sy  = np.array(slider_y) * 1e3
    ty  = np.array(tablet_y) * 1e3
    gap = np.array(gap_log)
    fe_mag  = np.linalg.norm(fe, axis=1)

    J_Y     = float(np.trapz(fe[:, 1], t))
    F_Y_max = float(fe[:, 1].max())
    F_Y_min = float(fe[:, 1].min())

    print(f"\n  {'='*60}")
    print(f"  수집    : {len(t)} steps  ({t[-1]:.2f} s)")
    print(f"  Slider Y range  : {sy.min():.1f} ~ {sy.max():.1f} mm")
    print(f"  Tablet Y range  : {ty.min():.1f} ~ {ty.max():.1f} mm")
    print(f"  Min gap         : {gap.min():.2f} mm  (<0 = penetration)")
    print(f"  F_Y range       : {F_Y_min:.3f} ~ {F_Y_max:.3f} N")
    print(f"  |F| max         : {fe_mag.max():.3f} N")
    print(f"  Impulse J_Y     : {J_Y:.5f} N·s")
    if first_contact_t:
        print(f"  First contact   : t = {first_contact_t:.3f} s")
    else:
        print("  [!] No contact detected")
    print(f"  {'='*60}")

    # ── 플롯 ─────────────────────────────────────────────────────────
    title_base = (f"Motor={MOTOR_CTRL} N·m  |  "
                  f"R={R_mm:.1f}mm AR={AR:.2f} CV={CV:.2f}")

    fig1, axes1 = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    fig1.suptitle(f"Position — {title_base}", fontsize=11, fontweight="bold")
    axes1[0].plot(t, sy, color="tab:orange", lw=1.5, label="Slider Y (L8)")
    axes1[0].plot(t, ty, color="tab:blue",   lw=1.5, label="Tablet Y (center)")
    axes1[0].axhline(WALL_Y_MM, color="tab:red", ls=":", lw=1.2,
                     label=f"Wall Y={WALL_Y_MM:.1f}mm")
    axes1[0].set_ylabel("World Y [mm]"); axes1[0].legend(fontsize=9)
    axes1[0].grid(True, alpha=0.3)
    axes1[1].plot(t, gap, color="tab:purple", lw=1.5)
    axes1[1].axhline(0, color="tab:red", ls="--", lw=0.8, label="Contact (gap=0)")
    axes1[1].set_ylabel("Gap [mm]"); axes1[1].set_xlabel("Time [s]")
    axes1[1].legend(fontsize=9); axes1[1].grid(True, alpha=0.3)
    fig1.tight_layout()

    fig2, axes2 = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    fig2.suptitle(f"Force F_Y & Impulse — {title_base}",
                  fontsize=11, fontweight="bold")
    axes2[0].plot(t, fe[:, 1], color="tab:blue",   lw=1.5, label="cfrc_ext F_Y")
    axes2[0].plot(t, fs[:, 1], color="tab:orange", lw=1.0, ls="--",
                  alpha=0.75, label="site sensor F_Y")
    axes2[0].fill_between(t, 0, fe[:, 1], where=(fe[:, 1] > 0),
                           alpha=0.12, color="tab:blue")
    axes2[0].fill_between(t, 0, fe[:, 1], where=(fe[:, 1] < 0),
                           alpha=0.12, color="tab:red")
    axes2[0].set_ylabel("F_Y [N]")
    axes2[0].set_title(f"max={F_Y_max:.3f} N  min={F_Y_min:.3f} N")
    axes2[0].legend(fontsize=9); axes2[0].grid(True, alpha=0.3)
    J_cumul = np.cumsum(fe[:, 1]) * float(model.opt.timestep)
    axes2[1].plot(t, J_cumul, color="tab:green", lw=1.5,
                  label=f"J_Y = {J_Y:.4f} N·s")
    axes2[1].set_ylabel("J_Y [N·s]"); axes2[1].set_xlabel("Time [s]")
    axes2[1].legend(fontsize=9); axes2[1].grid(True, alpha=0.3)
    fig2.tight_layout()

    fig3, axes3 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig3.suptitle("Force Components (cfrc_ext, world frame)",
                  fontsize=11, fontweight="bold")
    for i, (lbl, col) in enumerate(
            [("X (lateral)", "tab:red"),
             ("Y (normal / compression)", "tab:blue"),
             ("Z (vertical)", "tab:green")]):
        axes3[i].plot(t, fe[:, i], color=col, lw=1.2, label=f"F_{lbl}")
        axes3[i].axhline(0, color="k", lw=0.5)
        axes3[i].set_ylabel(f"F [N]"); axes3[i].legend(fontsize=9)
        axes3[i].grid(True, alpha=0.3)
    axes3[2].set_xlabel("Time [s]")
    fig3.tight_layout()

    # ── 저장 ─────────────────────────────────────────────────────────
    for fig, name in [
        (fig1, "crusher_tablet_position.png"),
        (fig2, "crusher_tablet_force_magnitude.png"),
        (fig3, "crusher_tablet_force_components.png"),
    ]:
        p = os.path.join(_HERE, name)
        fig.savefig(p, dpi=150, bbox_inches="tight")
        print(f"  saved: {name}")
    plt.show()


# ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Crusher + Tablet 2-Phase 통합 시뮬레이션",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="예시:\n  python crusher_tablet_sim.py tablet_R6.0_AR1.50_CV0.20.stl",
    )
    parser.add_argument("stl", nargs="?", default=None,
                        help="Tablet STL 파일 경로")
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
        print("사용법: python crusher_tablet_sim.py <path>.stl")
        sys.exit(0)

    stl_path = os.path.abspath(stl_path)
    if not os.path.exists(stl_path):
        print(f"[오류] 파일 없음: {stl_path}")
        sys.exit(1)

    run(stl_path)

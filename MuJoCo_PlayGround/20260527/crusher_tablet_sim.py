"""
crusher_tablet_sim.py  [v4 — mocap tablet]
Crusher + Tablet 통합 시뮬레이션

▶ 알약 고정 방식: mocap body
    - freejoint 없음 → 관통 없음
    - data.mocap_pos / data.mocap_quat 으로 위치 제어
    - 충돌력은 data.contact + mj_contactForce 로 수집

▶ Phase 1  (PHASE1_STEPS 스텝, 뷰어 없음)
    lock_crank equality 활성 (-90°) + mocap 알약 배치 → 메커니즘 안정화

▶ Phase 2  (뷰어 오픈)
    MOTOR_DELAY 초 후 lock_crank 해제 → Motor CCW 구동 → 접촉력 기록

▶ 배치 좌표 (MuJoCo world frame)
    PLACE_X_MM = -47.879  →  MuJoCo X
    PLACE_Z_MM =  50.108  →  MuJoCo Z
    WALL_Y_MM  = 336.199  →  MuJoCo Y  (충돌판 벽, 알약 중심)

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
WALL_Y_MM  = 336.199   # impact plate 벽 위치 (= 알약 중심 Y)

# ── Phase 1 스텝 수 (뷰어 없이 안정화) ──────────────────────────────
PHASE1_STEPS = 500

# ── Phase 2 시뮬레이션 파라미터 ──────────────────────────────────────
SIM_DURATION = 30.0   # 측정 시간 [s]
MOTOR_CTRL   = -0.5   # Motor1_crank 제어 입력 [N·m]  (음수 = CCW)
MOTOR_DELAY  =  3.0   # 모터 구동 지연 시간 [s]  (Phase 2 시작 후)

# ── 알약 초기 자세 (쿼터니언) ────────────────────────────────────────
# local-Z(두께방향) → world-Y(압축방향): X축 기준 90° 회전
TAB_QUAT = np.array([0.7071068, 0.7071068, 0.0, 0.0])   # [qw, qx, qy, qz]


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

      ① meshdir  → Crusher MJCF 디렉토리 절대경로
      ② keyframe 제거  → nq 불일치 방지
      ③ tablet body    → mocap="true" + geom  (freejoint/site/sensor 없음)
         mocap body는 joint가 없으므로 nq 변경 없음
    """
    pos_x = PLACE_X_MM * 1e-3
    pos_z = PLACE_Z_MM * 1e-3
    pos_y = WALL_Y_MM  * 1e-3   # 알약 중심 = 벽면

    print(f"  배치 [mm] : X={PLACE_X_MM:.3f}  Y={WALL_Y_MM:.3f}  Z={PLACE_Z_MM:.3f}")
    print(f"  배치 [m]  : X={pos_x:.5f}  Y={pos_y:.5f}  Z={pos_z:.5f}")

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

    # lock_crank equality 는 유지 → 런타임에 data.eq_active 로 해제

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
    #   joint 없음: nq 그대로, 위치는 data.mocap_pos 로 제어
    worldbody = root.find("worldbody")
    tab = ET.SubElement(worldbody, "body", {
        "name":  "tablet",
        "mocap": "true",
        "pos":   f"{pos_x:.6f} {pos_y:.6f} {pos_z:.6f}",
        "quat":  f"{TAB_QUAT[0]} {TAB_QUAT[1]} {TAB_QUAT[2]} {TAB_QUAT[3]}",
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
    부호 규칙: geom1 body 기준으로 반환되므로 geom2가 body_id면 부호 반전.
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
        # contact frame 행렬 (행 = contact frame 축, world 좌표)
        frame   = c.frame.reshape(3, 3)
        f_world = frame.T @ force6[:3]   # contact → world
        if g2_b == body_id:
            f_world = -f_world           # 부호: geom2 body 기준으로 반전
        f_total += f_world
    return f_total


# ─────────────────────────────────────────────────────────────────────
def run(stl_path: str):
    print("=" * 62)
    print("  Crusher + Tablet  2-Phase 통합 시뮬레이션  [mocap 알약]")
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
    mocap_id   = model.body_mocapid[b_tablet]   # mocap 배열 인덱스

    print(f"\n  nq={model.nq}  |  crank qpos[{crank_qadr}]  "
          f"mocap_id={mocap_id}  lock_crank eq_id={eq_lock_id}")

    # ── ❶ 초기 상태 설정 ────────────────────────────────────────────
    data.qpos[crank_qadr] = -np.pi / 2   # 크랭크 -90°
    data.qvel[:]          = 0.0
    # mocap 알약 위치·자세 지정
    data.mocap_pos[mocap_id]  = [px, py, pz]
    data.mocap_quat[mocap_id] = TAB_QUAT
    mujoco.mj_forward(model, data)

    # ── ❷ Phase 1: 메커니즘 안정화 (알약은 mocap이 자동 고정) ────────
    print(f"\n◆ Phase 1: {PHASE1_STEPS} 스텝 안정화 (뷰어 없음)")
    print(f"  tablet mocap 위치: ({px:.4f}, {py:.4f}, {pz:.4f}) m")

    for _ in range(PHASE1_STEPS):
        mujoco.mj_step(model, data)
        # mocap body는 mj_step 후에도 mocap_pos 값을 유지
        # (별도 덮어쓰기 불필요)

    mujoco.mj_forward(model, data)
    crank_deg = np.degrees(data.qpos[crank_qadr])
    print(f"  ✔ Phase 1 완료  crank={crank_deg:.1f}°  sim_time={data.time:.3f}s")

    # ── ❸ Phase 2: lock 유지 → MOTOR_DELAY 후 해제 + CCW 구동 ────────
    print(f"\n◆ Phase 2: 뷰어 오픈  (lock_crank 활성)")
    print(f"  {MOTOR_DELAY:.1f}s 후 lock_crank 해제 → {MOTOR_CTRL} N·m CCW")
    print(f"  측정 시간 = {SIM_DURATION} s\n")

    data.ctrl[act_crank] = 0.0
    phase2_start_t = data.time
    motor_on       = False

    t_log    = []
    f_log    = []   # contact force (world XYZ)
    slider_y = []
    tablet_y = []
    gap_log  = []
    first_contact_t = None

    print(f"  {'Time':>6s} | {'Slider_Y':>9s} mm | {'Tablet_Y':>9s} mm | "
          f"{'Gap':>7s} mm | {'F_Y':>8s} N | {'ncon':>4s}")
    print("  " + "-" * 72)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT]      = False
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = False
        viewer.opt.frame                                      = mujoco.mjtFrame.mjFRAME_NONE
        viewer.opt.geomgroup[3]                               = False
        for sg in range(5):
            viewer.opt.sitegroup[sg] = False

        while viewer.is_running() and data.time < SIM_DURATION:

            # MOTOR_DELAY 후 lock 해제 + 모터 ON
            if not motor_on and (data.time - phase2_start_t) >= MOTOR_DELAY:
                data.eq_active[eq_lock_id] = 0
                data.ctrl[act_crank]       = MOTOR_CTRL
                motor_on = True
                print(f"  *** lock_crank 해제 + 모터 ON: t={data.time:.3f}s "
                      f"ctrl={MOTOR_CTRL} N·m (CCW) ***")

            mujoco.mj_step(model, data)
            # mocap body: mj_step 후 위치 자동 유지 (덮어쓰기 불필요)

            sy     = float(data.xpos[b_slider, 1])
            ty_now = float(data.xpos[b_tablet, 1])
            gap_mm = (ty_now - sy) * 1e3

            # ── 접촉력 집계 (contact frame → world frame) ────────────
            fc_now = _sum_contact_force(model, data, b_tablet)

            t_log.append(data.time)
            f_log.append(fc_now.copy())
            slider_y.append(sy)
            tablet_y.append(ty_now)
            gap_log.append(gap_mm)

            # 첫 접촉 감지
            if first_contact_t is None and data.ncon > 0:
                # 알약과 관련된 접촉이 있는지 확인
                for ci in range(data.ncon):
                    c = data.contact[ci]
                    if b_tablet in (model.geom_bodyid[c.geom1],
                                    model.geom_bodyid[c.geom2]):
                        first_contact_t = data.time
                        print(f"  *** 첫 접촉: t={data.time:.3f}s  "
                              f"F_Y={fc_now[1]:.2f}N  gap={gap_mm:.2f}mm ***")
                        break

            # 500 스텝마다 콘솔 출력
            if len(t_log) % 500 == 0:
                print(f"  {data.time:6.2f}s | {sy*1e3:9.2f}    | "
                      f"{ty_now*1e3:9.2f}    | {gap_mm:7.2f}    | "
                      f"{fc_now[1]:8.3f} | {data.ncon:4d}")

            viewer.sync()

    # ── 결과 집계 ─────────────────────────────────────────────────────
    if not t_log:
        print("[경고] 데이터 없음.")
        return

    t   = np.array(t_log)
    fc  = np.array(f_log)          # (N, 3) world frame
    sy  = np.array(slider_y) * 1e3
    ty  = np.array(tablet_y) * 1e3
    gap = np.array(gap_log)
    fc_mag  = np.linalg.norm(fc, axis=1)

    J_Y     = float(np.trapz(fc[:, 1], t))
    F_Y_max = float(fc[:, 1].max())
    F_Y_min = float(fc[:, 1].min())

    print(f"\n  {'='*60}")
    print(f"  수집    : {len(t)} steps  ({t[-1]:.2f} s)")
    print(f"  Slider Y range  : {sy.min():.1f} ~ {sy.max():.1f} mm")
    print(f"  Tablet Y range  : {ty.min():.1f} ~ {ty.max():.1f} mm")
    print(f"  Min gap         : {gap.min():.2f} mm  (<0 = penetration)")
    print(f"  F_Y range       : {F_Y_min:.3f} ~ {F_Y_max:.3f} N")
    print(f"  |F| max         : {fc_mag.max():.3f} N")
    print(f"  Impulse J_Y     : {J_Y:.5f} N·s")
    if first_contact_t:
        print(f"  First contact   : t = {first_contact_t:.3f} s")
    else:
        print("  [!] No contact detected")
    print(f"  {'='*60}")

    # ── 플롯 ─────────────────────────────────────────────────────────
    title_base = (f"Motor={MOTOR_CTRL} N·m (CCW)  |  "
                  f"R={R_mm:.1f}mm AR={AR:.2f} CV={CV:.2f}")

    # 그림 1: 위치
    fig1, axes1 = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    fig1.suptitle(f"Position — {title_base}", fontsize=11, fontweight="bold")
    axes1[0].plot(t, sy, color="tab:orange", lw=1.5, label="Slider Y (L8)")
    axes1[0].plot(t, ty, color="tab:blue",   lw=1.5, label="Tablet Y (mocap)")
    axes1[0].axhline(WALL_Y_MM, color="tab:red", ls=":", lw=1.2,
                     label=f"Wall Y={WALL_Y_MM:.1f}mm")
    axes1[0].set_ylabel("World Y [mm]"); axes1[0].legend(fontsize=9)
    axes1[0].grid(True, alpha=0.3)
    axes1[1].plot(t, gap, color="tab:purple", lw=1.5)
    axes1[1].axhline(0, color="tab:red", ls="--", lw=0.8, label="Contact (gap=0)")
    axes1[1].set_ylabel("Gap [mm]"); axes1[1].set_xlabel("Time [s]")
    axes1[1].legend(fontsize=9); axes1[1].grid(True, alpha=0.3)
    fig1.tight_layout()

    # 그림 2: F_Y + 임펄스
    fig2, axes2 = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    fig2.suptitle(f"Contact Force F_Y & Impulse — {title_base}",
                  fontsize=11, fontweight="bold")
    axes2[0].plot(t, fc[:, 1], color="tab:blue", lw=1.5, label="Contact F_Y (world)")
    axes2[0].fill_between(t, 0, fc[:, 1], where=(fc[:, 1] > 0),
                           alpha=0.12, color="tab:blue")
    axes2[0].fill_between(t, 0, fc[:, 1], where=(fc[:, 1] < 0),
                           alpha=0.12, color="tab:red")
    axes2[0].set_ylabel("F_Y [N]")
    axes2[0].set_title(f"max={F_Y_max:.3f} N  min={F_Y_min:.3f} N")
    axes2[0].legend(fontsize=9); axes2[0].grid(True, alpha=0.3)
    J_cumul = np.cumsum(fc[:, 1]) * float(model.opt.timestep)
    axes2[1].plot(t, J_cumul, color="tab:green", lw=1.5,
                  label=f"J_Y = {J_Y:.4f} N·s")
    axes2[1].set_ylabel("J_Y [N·s]"); axes2[1].set_xlabel("Time [s]")
    axes2[1].legend(fontsize=9); axes2[1].grid(True, alpha=0.3)
    fig2.tight_layout()

    # 그림 3: XYZ 성분
    fig3, axes3 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig3.suptitle("Contact Force Components (world frame)",
                  fontsize=11, fontweight="bold")
    for i, (lbl, col) in enumerate(
            [("X (lateral)",              "tab:red"),
             ("Y (normal / compression)", "tab:blue"),
             ("Z (vertical)",             "tab:green")]):
        axes3[i].plot(t, fc[:, i], color=col, lw=1.2, label=f"F_{lbl}")
        axes3[i].axhline(0, color="k", lw=0.5)
        axes3[i].set_ylabel("F [N]"); axes3[i].legend(fontsize=9)
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
        description="Crusher + Tablet 2-Phase 통합 시뮬레이션 (mocap 알약)",
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

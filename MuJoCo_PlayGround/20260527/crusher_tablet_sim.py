"""
crusher_tablet_sim.py
Crusher + Tablet 통합 시뮬레이션 — 힘 센서 실시간 기록

▶ 동작 방식
  - Crusher_IsaacSim_colored.xml 과 Tablet STL 을 Python 에서 독립적으로 로드
  - XML 파일 자체는 수정하지 않음 — 메모리에서만 조합
  - Tablet 을 지정 위치에 배치 후 Motor1_crank 구동
  - 시뮬레이션 중 tablet 에 작용하는 외부 접촉력을 실시간 기록 + 콘솔 출력
  - 뷰어 종료 후 matplotlib 플롯 저장

▶ 태블릿 배치 위치 (world frame, mm)
  X = -47.879
  Y = 336.199 − tablet_두께/2   (far face = Y 336.199mm 고정)
  Z =  50.108

▶ 태블릿 방향
  local Z (Fusion 360 두께 방향) → world Y (슬라이더 압축 방향)
  quat = (0.7071, 0.7071, 0, 0)  — world X 축 기준 90° 회전

▶ 고정 뒷벽
  Crusher XML 의 L1_Wall1_1 (worldbody 고정 geom, contype/conaffinity=1) 이
  Y≈WALL_Y_MM 위치에 이미 존재 → 알약은 L8 impact plate 와 L1_Wall1_1 사이에서 압축됨.
  별도 back_wall 주입 불필요.

▶ 힘 측정
  data.cfrc_ext[tablet_body, 3:6] : contact + applied force (world frame) [N]
  data.sensordata (force site 센서) : 비교용

▶ 실행
    conda activate isaac_sim
    python crusher_tablet_sim.py [tablet.stl]
    (인자 없으면 파일 선택 다이얼로그)
"""

import os
import sys
import argparse
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer

# ── 경로 ─────────────────────────────────────────────────────────────
_HERE     = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.normpath(os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml"))
MJCF_DIR  = os.path.dirname(MJCF_PATH)
STL_DIR   = os.path.normpath(os.path.join(_HERE, "..", "..", "tablets_stl", "stl"))

# ── 시뮬레이션 파라미터 ───────────────────────────────────────────────
SIM_DURATION = 30.0   # 측정 시간 [s]
MOTOR_CTRL   = 0.5    # Motor1_crank 제어 입력 [N·m]

# ── 태블릿 배치 위치 (mm) ─────────────────────────────────────────────
PLACE_X_MM = -47.879
PLACE_Z_MM =  50.108
WALL_Y_MM  = 336.199  # 태블릿 far face (슬라이더 반대쪽) Y 위치


# ─────────────────────────────────────────────────────────────────────
def _tablet_thickness_mm(stl_path: str) -> float:
    """STL bounding box 에서 두께(Z 방향) 추출 [mm]."""
    try:
        import trimesh
        m = trimesh.load(stl_path, force="mesh")
        return float(m.bounds[1, 2] - m.bounds[0, 2])
    except Exception:
        # trimesh 없거나 로드 실패 시 파일명 파라미터로 추정
        import re
        stem = os.path.splitext(os.path.basename(stl_path))[0]
        hit  = re.search(r"R([\d.]+)_AR([\d.]+)_CV([\d.]+)", stem)
        if hit:
            R, CV = float(hit.group(1)), float(hit.group(3))
            cd = CV * 2 * R
            return R * 0.20 + 2 * cd   # bh + 2*cd
        return 8.0  # fallback


def _build_model(stl_path: str):
    """
    Crusher XML + Tablet STL 을 메모리에서 조합해 MjModel 반환.

    1. Crusher XML 을 ET 로 파싱
    2. compiler 에 meshdir 추가 (Crusher STL 위치 지정)
    3. Tablet mesh asset / body / site / sensor 주입
    4. from_xml_string(xml, assets={'tablet.stl': bytes}) 로 로드
    """
    thickness_mm = _tablet_thickness_mm(stl_path)
    thickness_m  = thickness_mm * 1e-3

    # 태블릿 중심 world 좌표 [m]
    tx = PLACE_X_MM * 1e-3
    ty = (WALL_Y_MM - thickness_mm / 2.0) * 1e-3
    tz = PLACE_Z_MM * 1e-3

    print(f"  STL           : {os.path.basename(stl_path)}")
    print(f"  두께          : {thickness_mm:.2f} mm")
    print(f"  배치 중심     : ({tx*1e3:.3f}, {ty*1e3:.3f}, {tz*1e3:.3f}) mm")
    print(f"  Far face Y    : {WALL_Y_MM:.3f} mm  (wall side)")

    # ── Crusher XML 파싱 ─────────────────────────────────────────────
    tree = ET.parse(MJCF_PATH)
    root = tree.getroot()

    # meshdir 설정 — Crusher 부품 STL 을 절대경로로 참조
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", MJCF_DIR)

    # ── Tablet mesh asset 추가 ────────────────────────────────────────
    asset = root.find("asset")
    ET.SubElement(asset, "mesh", {
        "name": "tablet_mesh",
        "file": "tablet.stl",        # assets dict 에서 공급
        "scale": ".001 .001 .001",
    })
    ET.SubElement(asset, "material", {
        "name": "tablet_mat",
        "rgba": ".85 .80 .72 1",
        "specular": ".4",
        "shininess": ".3",
    })

    # ── Tablet body → worldbody ──────────────────────────────────────
    worldbody = root.find("worldbody")
    tab = ET.SubElement(worldbody, "body", {
        "name": "tablet",
        "pos": f"{tx:.6f} {ty:.6f} {tz:.6f}",
        # local Z(두께) → world Y(슬라이더 방향): X축 기준 90° 회전
        "quat": "0.7071068 0.7071068 0 0",
    })
    ET.SubElement(tab, "freejoint", {"name": "tablet_free"})
    ET.SubElement(tab, "site", {
        "name": "tablet_force_site",
        "pos": "0 0 0",
        "size": "0.005",
    })
    ET.SubElement(tab, "geom", {
        "name":     "tablet_geom",
        "type":     "mesh",
        "mesh":     "tablet_mesh",
        "material": "tablet_mat",
        "density":  "1200",       # 경구정 밀도 [kg/m³]
        "condim":   "4",
        "friction": ".5 .02 .01",
    })

    # ── 참고: 뒷면 고정 벽 ──────────────────────────────────────────────
    # Crusher XML 의 L1_Wall1_1 (worldbody 고정 geom, contype=1/conaffinity=1) 이
    # 이미 Y≈WALL_Y_MM 위치에 존재하므로 별도 back_wall 주입 불필요.
    # 알약은 L8 impact plate 와 L1_Wall1_1 사이에서 압축됨.

    # ── Force / Torque 센서 (actuator 뒤에 추가) ─────────────────────
    sensor_sec = ET.SubElement(root, "sensor")
    ET.SubElement(sensor_sec, "force",  {"name": "tablet_force",  "site": "tablet_force_site"})
    ET.SubElement(sensor_sec, "torque", {"name": "tablet_torque", "site": "tablet_force_site"})

    # ── 직렬화 후 로드 ────────────────────────────────────────────────
    xml_str   = ET.tostring(root, encoding="unicode")
    stl_bytes = open(stl_path, "rb").read()
    model     = mujoco.MjModel.from_xml_string(xml_str, assets={"tablet.stl": stl_bytes})
    return model, thickness_m


# ─────────────────────────────────────────────────────────────────────
def run(stl_path: str):
    print("=" * 58)
    print("  Crusher + Tablet 통합 시뮬레이션 — 힘 센서")
    print("=" * 58)

    model, thickness_m = _build_model(stl_path)
    data = mujoco.MjData(model)

    # ── ID 조회 ──────────────────────────────────────────────────────
    b_tablet  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,    "tablet")
    b_slider  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,    "L8_Link3_Shaft_1")
    act_crank = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Motor1_crank")
    s_id      = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR,   "tablet_force")
    s_adr     = model.sensor_adr[s_id]

    tablet_init_y = (WALL_Y_MM - thickness_m * 1e3 / 2.0) * 1e-3  # [m]

    # ── 데이터 버퍼 ──────────────────────────────────────────────────
    t_log    = []
    f_ext    = []   # cfrc_ext[tablet, 3:6] — contact force (world frame) [N]
    f_sens   = []   # force site sensor output (비교용) [N]
    slider_y = []   # 슬라이더 body world Y [m]
    tablet_y = []   # 알약 body world Y [m]
    gap_log  = []   # 슬라이더 → 알약 near-face 간격 [mm]

    # ── 모터 ON ───────────────────────────────────────────────────────
    data.ctrl[act_crank] = MOTOR_CTRL
    first_contact_t = None

    print(f"\n  Motor1_crank    : {MOTOR_CTRL} N·m")
    print(f"  측정 시간       : {SIM_DURATION} s")
    print(f"  알약 초기 Y     : {tablet_init_y*1e3:.2f} mm")
    print(f"  L1_Wall1_1 Y    : {WALL_Y_MM:.2f} mm (Crusher 고정 뒷벽)")
    print("\n  뷰어 종료 시 플롯이 자동 저장됩니다.\n")
    print(f"  {'Time':>6s} | {'Slider_Y':>9s} mm | {'Tablet_Y':>9s} mm | "
          f"{'Gap':>7s} mm | {'F_Y':>8s} N  | {'|F|':>7s} N")
    print("  " + "-" * 70)

    # ── 시뮬레이션 루프 ──────────────────────────────────────────────
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = False
        viewer.opt.geomgroup[3] = False

        while viewer.is_running() and data.time < SIM_DURATION:
            mujoco.mj_step(model, data)

            sy = float(data.xpos[b_slider, 1])
            ty = float(data.xpos[b_tablet, 1])
            gap_mm = (ty - sy) * 1e3

            fe_now = data.cfrc_ext[b_tablet, 3:6].copy()
            fs_now = data.sensordata[s_adr : s_adr + 3].copy()

            t_log.append(data.time)
            slider_y.append(sy)
            tablet_y.append(ty)
            gap_log.append(gap_mm)
            f_ext.append(fe_now)
            f_sens.append(fs_now)

            # 첫 접촉 감지 (|F_Y| > 0.1 N)
            if first_contact_t is None and abs(fe_now[1]) > 0.1:
                first_contact_t = data.time
                print(f"  *** 첫 접촉: t={data.time:.3f}s  "
                      f"F_Y={fe_now[1]:.2f}N  gap={gap_mm:.2f}mm ***")

            # 실시간 콘솔 (250 스텝 = 0.5s 마다)
            if len(t_log) % 250 == 0:
                mag = np.linalg.norm(fe_now)
                print(f"  {data.time:6.2f}s | {sy*1e3:9.2f}    | {ty*1e3:9.2f}    | "
                      f"{gap_mm:7.2f}    | {fe_now[1]:8.3f}    | {mag:7.3f}")

            viewer.sync()

    # ── numpy 변환 ───────────────────────────────────────────────────
    t   = np.array(t_log)
    fe  = np.array(f_ext)
    fs  = np.array(f_sens)
    sy  = np.array(slider_y) * 1e3   # [mm]
    ty  = np.array(tablet_y) * 1e3   # [mm]
    gap = np.array(gap_log)

    if len(t) == 0:
        print("[경고] 데이터 없음.")
        return

    fe_mag  = np.linalg.norm(fe, axis=1)
    J_Y     = float(np.trapz(fe[:, 1], t))
    F_Y_max = float(fe[:, 1].max())
    F_Y_min = float(fe[:, 1].min())

    print(f"\n  {'='*58}")
    print(f"  수집 완료       : {len(t)} steps  ({t[-1]:.2f} s)")
    print(f"  Slider Y range  : {sy.min():.1f} ~ {sy.max():.1f} mm")
    print(f"  Tablet Y range  : {ty.min():.1f} ~ {ty.max():.1f} mm")
    print(f"  Min gap         : {gap.min():.2f} mm  (<0 = penetration = contact)")
    print(f"  F_Y range       : {F_Y_min:.3f} ~ {F_Y_max:.3f} N")
    print(f"  |F| max         : {fe_mag.max():.3f} N")
    print(f"  Impulse J_Y     : {J_Y:.5f} N·s  (= integral F_Y dt = delta mv_Y)")
    if first_contact_t:
        print(f"  First contact   : t = {first_contact_t:.3f} s")
    else:
        print("  [!] No contact detected — slider may not have reached tablet")
    print(f"  {'='*58}")

    # ── 플롯 1: 위치 추적 ────────────────────────────────────────────
    fig1, axes1 = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    fig1.suptitle(
        f"Slider & Tablet Position — Crusher Simulation\n"
        f"Motor={MOTOR_CTRL} N·m  |  STL={os.path.basename(stl_path)}",
        fontsize=12, fontweight="bold",
    )
    axes1[0].plot(t, sy, color="tab:orange", linewidth=1.5, label="Slider Y (L8 body)")
    axes1[0].plot(t, ty, color="tab:blue",   linewidth=1.5, label="Tablet Y (center)")
    axes1[0].axhline(WALL_Y_MM, color="tab:red", linestyle=":", linewidth=1.2,
                     label=f"L1_Wall1_1 back face Y = {WALL_Y_MM:.1f} mm")
    axes1[0].set_ylabel("World Y [mm]")
    axes1[0].set_title("Y Position over Time")
    axes1[0].legend(fontsize=9)
    axes1[0].grid(True, alpha=0.3)

    axes1[1].plot(t, gap, color="tab:purple", linewidth=1.5)
    axes1[1].axhline(0, color="tab:red", linestyle="--", linewidth=0.8, label="Contact (gap=0)")
    axes1[1].set_ylabel("Gap [mm]")
    axes1[1].set_xlabel("Time [s]")
    axes1[1].set_title("Slider to Tablet Near-Face Gap")
    axes1[1].legend(fontsize=9)
    axes1[1].grid(True, alpha=0.3)
    fig1.tight_layout()

    # ── 플롯 2: Y방향 Normal Force + Impulse ─────────────────────────
    fig2, axes2 = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    fig2.suptitle(
        f"Tablet Normal Force (Y) & Cumulative Impulse\n"
        f"Motor={MOTOR_CTRL} N·m  |  L1_Wall1_1 back face Y={WALL_Y_MM:.1f} mm",
        fontsize=12, fontweight="bold",
    )
    axes2[0].plot(t, fe[:, 1], color="tab:blue", linewidth=1.5,
                  label="F_Y  cfrc_ext  (contact, world frame)")
    axes2[0].plot(t, fs[:, 1], color="tab:orange", linewidth=1.0, linestyle="--",
                  alpha=0.75, label="F_Y  force site sensor")
    axes2[0].axhline(0, color="k", linewidth=0.5)
    axes2[0].fill_between(t, 0, fe[:, 1], where=(fe[:, 1] > 0),
                          alpha=0.15, color="tab:blue",  label="Compression (+Y)")
    axes2[0].fill_between(t, 0, fe[:, 1], where=(fe[:, 1] < 0),
                          alpha=0.15, color="tab:red",   label="Tension (-Y)")
    axes2[0].set_ylabel("F_Y [N]")
    axes2[0].set_title(f"Normal Force F_Y  (max={F_Y_max:.3f} N,  min={F_Y_min:.3f} N)")
    axes2[0].legend(fontsize=9)
    axes2[0].grid(True, alpha=0.3)

    J_cumul = np.cumsum(fe[:, 1]) * float(model.opt.timestep)
    axes2[1].plot(t, J_cumul, color="tab:green", linewidth=1.5,
                  label=f"Cumul. Impulse J_Y = {J_Y:.4f} N·s")
    axes2[1].axhline(0, color="k", linewidth=0.5)
    axes2[1].set_ylabel("J_Y [N·s]")
    axes2[1].set_xlabel("Time [s]")
    axes2[1].set_title("Cumulative Impulse  J_Y = integral(F_Y dt)  =  delta(m * v_Y)")
    axes2[1].legend(fontsize=9)
    axes2[1].grid(True, alpha=0.3)
    fig2.tight_layout()

    # ── 플롯 3: 3축 성분 ─────────────────────────────────────────────
    fig3, axes3 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig3.suptitle("Tablet Force Components  (cfrc_ext, world frame)",
                  fontsize=12, fontweight="bold")
    axis_info = [
        ("X", "tab:red",   "Lateral force"),
        ("Y", "tab:blue",  "Normal force  (compression direction)"),
        ("Z", "tab:green", "Vertical force  (gravity + floor)"),
    ]
    for i, (lbl, col, desc) in enumerate(axis_info):
        axes3[i].plot(t, fe[:, i], color=col, linewidth=1.2, label=f"F_{lbl}  [{desc}]")
        axes3[i].axhline(0, color="k", linewidth=0.5)
        axes3[i].set_ylabel(f"F_{lbl} [N]")
        axes3[i].legend(fontsize=9, loc="upper right")
        axes3[i].grid(True, alpha=0.3)
    axes3[2].set_xlabel("Time [s]")
    fig3.tight_layout()

    # ── 저장 ─────────────────────────────────────────────────────────
    paths = {
        "position":     os.path.join(_HERE, "crusher_tablet_position.png"),
        "normal_force": os.path.join(_HERE, "crusher_tablet_normal_force.png"),
        "components":   os.path.join(_HERE, "crusher_tablet_force_components.png"),
    }
    fig1.savefig(paths["position"],     dpi=150, bbox_inches="tight")
    fig2.savefig(paths["normal_force"], dpi=150, bbox_inches="tight")
    fig3.savefig(paths["components"],   dpi=150, bbox_inches="tight")
    for name, p in paths.items():
        print(f"  saved [{name}]: {os.path.basename(p)}")
    plt.show()


# ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Crusher + Tablet 통합 시뮬레이션",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="예시:\n  python crusher_tablet_sim.py tablet_R6.0_AR1.50_CV0.20.stl",
    )
    parser.add_argument("stl", nargs="?", default=None, help="Tablet STL 파일 경로")
    args = parser.parse_args()

    stl_path = args.stl

    # 인자 없으면 파일 선택 다이얼로그
    if stl_path is None:
        try:
            import tkinter as tk
            from tkinter import filedialog
            root_tk = tk.Tk()
            root_tk.withdraw()
            stl_path = filedialog.askopenfilename(
                title="Tablet STL 선택",
                initialdir=STL_DIR if os.path.isdir(STL_DIR) else os.path.expanduser("~"),
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

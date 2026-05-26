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
SIM_DURATION = 10.0   # 측정 시간 [s]
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

    model, _ = _build_model(stl_path)
    data      = mujoco.MjData(model)

    # ID 조회
    b_tablet  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,     "tablet")
    act_crank = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,  "Motor1_crank")
    s_id      = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR,    "tablet_force")
    s_adr     = model.sensor_adr[s_id]   # sensordata 배열 시작 인덱스

    # ── 데이터 버퍼 ──────────────────────────────────────────────────
    t_log  = []
    f_ext  = []   # cfrc_ext[tablet, 3:6] — 외부 접촉력 (world frame)
    f_sens = []   # sensordata 의 force site 출력 (비교용)

    # ── 모터 ON ───────────────────────────────────────────────────────
    data.ctrl[act_crank] = MOTOR_CTRL

    print(f"\n  Motor1_crank  : {MOTOR_CTRL} N·m")
    print(f"  측정 시간     : {SIM_DURATION} s")
    print("  뷰어 종료 시 플롯이 자동 저장됩니다.\n")
    print(f"  {'Time':>6s}  |  {'|F_contact|':>12s} N")
    print("  " + "-" * 24)

    # ── 시뮬레이션 루프 ──────────────────────────────────────────────
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = False
        viewer.opt.geomgroup[3] = False

        while viewer.is_running() and data.time < SIM_DURATION:
            mujoco.mj_step(model, data)

            t_log.append(data.time)

            # 외부 접촉력: cfrc_ext[body, 3:6] = force (world frame) [N]
            # cfrc_ext[body, 0:3] = torque (world frame) [N·m]
            f_ext.append(data.cfrc_ext[b_tablet, 3:6].copy())

            # 비교용: force site 센서 출력 (m*a 기반, 중력 미포함)
            f_sens.append(data.sensordata[s_adr : s_adr + 3].copy())

            # 실시간 콘솔 출력 (50스텝 = 0.1s 마다)
            if len(t_log) % 50 == 0:
                mag = np.linalg.norm(f_ext[-1])
                print(f"  {data.time:6.2f}s  |  {mag:12.2f}")

            viewer.sync()

    # ── numpy 변환 ───────────────────────────────────────────────────
    t  = np.array(t_log)
    fe = np.array(f_ext)    # (N, 3)  contact force
    fs = np.array(f_sens)   # (N, 3)  sensor output

    if len(t) == 0:
        print("[경고] 수집된 데이터 없음.")
        return

    fe_mag = np.linalg.norm(fe, axis=1)
    fs_mag = np.linalg.norm(fs, axis=1)

    print(f"\n  수집 완료     : {len(t)} 스텝  ({t[-1]:.3f} s)")
    print(f"  |F|_max       : {fe_mag.max():.2f} N  (접촉력)")
    print(f"  F_Y max       : {fe[:, 1].max():.2f} N  (슬라이드 방향)")

    # ── 플롯 1: 성분별 힘 ────────────────────────────────────────────
    fig1, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=True)
    fig1.suptitle(
        f"Tablet Force Components — Crusher Simulation\n"
        f"Motor={MOTOR_CTRL} N·m  |  STL={os.path.basename(stl_path)}",
        fontsize=12, fontweight="bold",
    )
    axis_labels = ["X", "Y (slide)", "Z (vertical)"]
    colors      = ["tab:red", "tab:blue", "tab:green"]

    for i in range(3):
        axes[i, 0].plot(t, fe[:, i], color=colors[i])
        axes[i, 0].set_title(f"Contact Force F{axis_labels[i]}  [cfrc_ext, world frame]")
        axes[i, 0].set_ylabel("F [N]")

        axes[i, 1].plot(t, fs[:, i], color=colors[i], linestyle="--")
        axes[i, 1].set_title(f"Sensor Output F{axis_labels[i]}  [force site]")
        axes[i, 1].set_ylabel("F [N]")

    for ax in axes.flat:
        ax.grid(True, alpha=0.3)
        ax.axhline(0, color="k", linewidth=0.5)
        ax.set_xlabel("Time [s]")

    fig1.tight_layout()

    # ── 플롯 2: 합력 크기 ────────────────────────────────────────────
    fig2, ax2 = plt.subplots(figsize=(10, 4))
    ax2.plot(t, fe_mag, color="tab:blue",   linewidth=1.5, label="|F| cfrc_ext (contact force)")
    ax2.plot(t, fs_mag, color="tab:orange", linewidth=1.2, linestyle="--",
             alpha=0.8, label="|F| sensor (force site)")
    ax2.fill_between(t, 0, fe_mag, alpha=0.12, color="tab:blue")
    ax2.set_title("Tablet Force Magnitude |F| over Time", fontsize=12)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("|F| [N]")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    fig2.tight_layout()

    # ── 저장 ─────────────────────────────────────────────────────────
    p1 = os.path.join(_HERE, "crusher_tablet_force_components.png")
    p2 = os.path.join(_HERE, "crusher_tablet_force_magnitude.png")
    fig1.savefig(p1, dpi=150, bbox_inches="tight")
    fig2.savefig(p2, dpi=150, bbox_inches="tight")
    print(f"\n  저장: {p1}")
    print(f"  저장: {p2}")
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

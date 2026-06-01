"""
check_placement.py
배치 좌표 시각 검증 — Crusher 씬에 마커를 찍어 PLACE 좌표 확인

배치 좌표 (MuJoCo world frame, mm):
    PLACE_X_MM = -47.879   →  MuJoCo X
    PLACE_Z_MM =  50.108   →  MuJoCo Z
    WALL_Y_MM  = 336.199   →  MuJoCo Y  (impact plate 벽)

마커 구성:
    ● 주황 구  : 타겟 포인트  (PLACE_X, WALL_Y, PLACE_Z)
    ─ 빨강 선  : X 축 스캔 (WALL_Y, PLACE_Z 고정)
    ─ 파랑 선  : Y 축 스캔 (PLACE_X, PLACE_Z 고정)
    ─ 초록 선  : Z 축 스캔 (PLACE_X, WALL_Y 고정)

실행:
    conda activate isaac_sim
    python check_placement.py
"""

import os
import sys
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import mujoco.viewer

# ── 경로 ─────────────────────────────────────────────────────────────
_HERE     = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.normpath(
    os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml"))
MJCF_DIR  = os.path.dirname(MJCF_PATH)

# ── 배치 좌표 (mm → m) ───────────────────────────────────────────────
PLACE_X_MM = -47.879
PLACE_Z_MM =  50.108
WALL_Y_MM  = 336.199

px = PLACE_X_MM * 1e-3   # MuJoCo X [m]
py = WALL_Y_MM  * 1e-3   # MuJoCo Y [m]
pz = PLACE_Z_MM * 1e-3   # MuJoCo Z [m]

# ── 크로스헤어 반길이 [m] ────────────────────────────────────────────
HL = 0.15   # ±15 cm


def build_model():
    """Crusher XML 에 마커 site 를 주입한 MjModel 반환."""

    tree = ET.parse(MJCF_PATH)
    root = tree.getroot()

    # meshdir → MJCF 디렉토리 절대경로 (Crusher STL 참조용)
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", MJCF_DIR)

    # keyframe 제거 (Python 에서 qpos 직접 지정)
    for kf in root.findall("keyframe"):
        root.remove(kf)

    # ── 마커 site 주입 ────────────────────────────────────────────────
    worldbody = root.find("worldbody")

    def add_site(parent, name, site_type, rgba, **kwargs):
        attrib = {"name": name, "type": site_type, "rgba": rgba}
        attrib.update({k: str(v) for k, v in kwargs.items()})
        ET.SubElement(parent, "site", attrib)

    # ① 타겟 포인트 : 주황 구
    add_site(worldbody, "target_point", "sphere", "1.0 0.5 0.0 1.0",
             pos=f"{px:.5f} {py:.5f} {pz:.5f}", size="0.008")

    # ② X축 크로스헤어 : 빨강 캡슐
    add_site(worldbody, "line_x", "capsule", "1.0 0.15 0.15 0.9",
             fromto=f"{px-HL:.5f} {py:.5f} {pz:.5f}  "
                    f"{px+HL:.5f} {py:.5f} {pz:.5f}",
             size="0.003")

    # ③ Y축 크로스헤어 : 파랑 캡슐
    add_site(worldbody, "line_y", "capsule", "0.15 0.4 1.0 0.9",
             fromto=f"{px:.5f} {py-HL:.5f} {pz:.5f}  "
                    f"{px:.5f} {py+HL:.5f} {pz:.5f}",
             size="0.003")

    # ④ Z축 크로스헤어 : 초록 캡슐
    add_site(worldbody, "line_z", "capsule", "0.15 1.0 0.15 0.9",
             fromto=f"{px:.5f} {py:.5f} {pz-HL:.5f}  "
                    f"{px:.5f} {py:.5f} {pz+HL:.5f}",
             size="0.003")

    # ⑤ WALL_Y 평면 표시 : 얇은 노랑 사각 캡슐 (Y=WALL_Y 고정, XZ 평면)
    add_site(worldbody, "wall_plane_x", "capsule", "1.0 0.9 0.1 0.6",
             fromto=f"{px-HL:.5f} {py:.5f} {pz:.5f}  "
                    f"{px+HL:.5f} {py:.5f} {pz:.5f}",
             size="0.001")

    xml_str = ET.tostring(root, encoding="unicode")
    model   = mujoco.MjModel.from_xml_string(xml_str)
    return model


# ─────────────────────────────────────────────────────────────────────
def main():
    print("=" * 58)
    print("  Placement Marker — 좌표 시각 검증")
    print("=" * 58)
    print(f"  PLACE_X_MM = {PLACE_X_MM}  →  MuJoCo X = {px:+.5f} m")
    print(f"  WALL_Y_MM  = {WALL_Y_MM}  →  MuJoCo Y = {py:+.5f} m")
    print(f"  PLACE_Z_MM = {PLACE_Z_MM}   →  MuJoCo Z = {pz:+.5f} m")
    print()
    print(f"  마커 색상:")
    print(f"    ● 주황 구  : 타겟 포인트 ({px:.4f}, {py:.4f}, {pz:.4f})")
    print(f"    ─ 빨강 선  : X 축")
    print(f"    ─ 파랑 선  : Y 축  (WALL_Y 방향)")
    print(f"    ─ 초록 선  : Z 축")
    print()

    try:
        model = build_model()
    except Exception as e:
        print(f"[ERROR] 모델 로드 실패: {e}")
        sys.exit(1)

    data = mujoco.MjData(model)

    # 크랭크 90° 초기화 (lock_crank equality 가 유지해주지만 초기값도 맞춤)
    crank_jid  = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "L3_Bevel_GearBox_1_L4_Shaft_1")
    data.qpos[model.jnt_qposadr[crank_jid]] = np.pi / 2
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    # ── site world 위치 출력 (검증) ───────────────────────────────────
    for site_name in ["target_point", "line_x", "line_y", "line_z"]:
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if sid >= 0:
            wpos = data.site_xpos[sid]
            print(f"  site '{site_name}' world pos: "
                  f"({wpos[0]:.4f}, {wpos[1]:.4f}, {wpos[2]:.4f}) m")
    print()

    # ── 참고: 주요 body 위치 출력 ────────────────────────────────────
    for bname in ["L8_Link3_Shaft_1", "L2_Left_Wall1_1"]:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bname)
        if bid >= 0:
            wpos = data.xpos[bid]
            print(f"  body '{bname}'  world pos: "
                  f"({wpos[0]:.4f}, {wpos[1]:.4f}, {wpos[2]:.4f}) m")
    print()
    print("  뷰어 실행 중... (마커 색상으로 위치 확인)")

    # ── 뷰어 ─────────────────────────────────────────────────────────
    # 마커 확인용 → 물리 스텝 없이 정적 표시 (mj_forward 만 사용)
    # mj_step 을 호출하면 lock_crank 고강성 constraint 로 인해 QACC NaN 발생
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT]      = False
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = False
        viewer.opt.frame                                      = mujoco.mjtFrame.mjFRAME_NONE
        viewer.opt.geomgroup[3]                               = False
        for sg in range(5):
            viewer.opt.sitegroup[sg] = True    # site 표시 ON

        while viewer.is_running():
            # mj_step 대신 mj_forward: 물리 적분 없이 위치/방향만 갱신
            mujoco.mj_forward(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()

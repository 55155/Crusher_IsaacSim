"""
open_crusher_colored.py
Crusher_IsaacSim_colored.xml (Ground.stl 포함) 을 MuJoCo 뷰어로 엽니다.

변경사항:
  - L2_Linear_bush_1  : contype/conaffinity=0  → Collision-Free
  - L2_Left_Wall1_1   : CoACD 11-hull 충돌 메시 적용
  - front2 조명       : 전방 보조 조명 추가
  - collision mesh    : 뷰어 시작 시 Collision Hull 표시 활성화

실행:
    conda activate isaacsim
    python open_crusher_colored.py

뷰어 단축키:
  F1  : 도움말
  V   : 시각화 플래그 토글 (collision hull 포함)
  Tab : 그룹 토글 (group=3 CoACD hull 표시/숨김)
"""
import os
import mujoco
import mujoco.viewer

_HERE     = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.normpath(os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml"))

model = mujoco.MjModel.from_xml_path(MJCF_PATH)
data  = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # ── Collision Hull OFF → visual mesh만 표시 ────────────────────────────
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = False

    # ── group=3 (CoACD hull geom) 숨김 ────────────────────────────────────
    viewer.opt.geomgroup[3] = False

    print("MuJoCo 뷰어 실행 중...")
    print("  - Collision Hull 표시: OFF  (visual mesh 전용)")
    print("  - group=3 CoACD hull: 숨김")
    print("  - 전방 조명(front2): ON")

    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

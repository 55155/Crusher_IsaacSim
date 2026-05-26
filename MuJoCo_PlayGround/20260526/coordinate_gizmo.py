"""
coordinate_gizmo.py
MuJoCo 월드 좌표계 확인 스크립트

뷰어 원점에 XYZ 기즈모(월드 프레임 축)를 표시합니다.

MuJoCo 기본 좌표계:
  X (빨강) : 오른쪽  →
  Y (초록) : 앞쪽    (화면 안쪽)
  Z (파랑) : 위쪽  ↑   ← 중력 반대 방향

실행:
    conda activate isaacsim
    python coordinate_gizmo.py

뷰어 단축키:
  F   : 프레임 표시 토글 (기즈모 on/off)
  F1  : 도움말
"""
import os
import mujoco
import mujoco.viewer
import numpy as np

_HERE     = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.normpath(
    os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml")
)

model = mujoco.MjModel.from_xml_path(MJCF_PATH)
data  = mujoco.MjData(model)

# ── 좌표계 정보 출력 ─────────────────────────────────────────────────────
print("=" * 60)
print("  MuJoCo 월드 좌표계 (World Frame)")
print("=" * 60)
print("  X 축 (빨강 ─━): 오른쪽  →")
print("  Y 축 (초록 ─━): 앞쪽 / 화면 안쪽  ↗")
print("  Z 축 (파랑 ─━): 위쪽  ↑   (중력 = -Z 방향)")
print()
print(f"  중력 벡터 (model.opt.gravity) : {model.opt.gravity}")
print()
print("  ┌─────────────────────────────────────────────────┐")
print("  │  이 모델 geom의 quat = '0.5 0.5 0.5 0.5'        │")
print("  │  → URDF/SolidWorks 설계 좌표 → MuJoCo 변환     │")
print("  │                                                  │")
print("  │    설계(URDF) X  →  MuJoCo Y  (초록)           │")
print("  │    설계(URDF) Y  →  MuJoCo Z  (파랑, 위)       │")
print("  │    설계(URDF) Z  →  MuJoCo X  (빨강)           │")
print("  └─────────────────────────────────────────────────┘")
print("=" * 60)

# ── quat 검증 ───────────────────────────────────────────────────────────
w, x, y, z = 0.5, 0.5, 0.5, 0.5
R = np.array([
    [1 - 2*(y*y + z*z),  2*(x*y - w*z),     2*(x*z + w*y)],
    [2*(x*y + w*z),      1 - 2*(x*x + z*z), 2*(y*z - w*x)],
    [2*(x*z - w*y),      2*(y*z + w*x),     1 - 2*(x*x + y*y)],
])
print("  quat=(0.5,0.5,0.5,0.5) 회전 행렬 R (MuJoCo ← URDF):")
print(f"    R = {np.round(R).astype(int).tolist()}")
print()
print("  → MuJoCo X = URDF Z  (R의 1열)")
print("  → MuJoCo Y = URDF X  (R의 2열)")
print("  → MuJoCo Z = URDF Y  (R의 3열)")
print("=" * 60)

# ── 뷰어 실행 ────────────────────────────────────────────────────────────
with mujoco.viewer.launch_passive(model, data) as viewer:

    # 월드 좌표축 기즈모 표시 (원점에 XYZ 화살표)
    viewer.opt.frame = mujoco.mjtFrame.mjFRAME_WORLD

    # visual mesh 전용 (collision hull 숨김)
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = False
    viewer.opt.geomgroup[3] = False

    print("뷰어 실행 중...")
    print("  원점(0,0,0)에 XYZ 기즈모가 표시됩니다.")
    print("  뷰어에서 'F' 키 → 프레임 토글 / 뷰어 우클릭 메뉴에서 크기 조절")

    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

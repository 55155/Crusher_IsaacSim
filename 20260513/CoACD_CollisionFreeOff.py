"""
CoACD_CollisionFreeOff.py
=========================
Crushing_CollisionFree.usd 에서 특정 링크의 collision 을 선택적으로 활성화합니다.

  L2_Left_Wall1_1 : collisionEnabled=True  +  approximation=convexDecomposition (CoACD)
  L2_Wall3_1      : collisionEnabled=True  +  approximation=convexHull  (단순 박스형)

변경 대상 속성 (collisions/.../node_STL_BINARY_ Xform):
  physics:collisionEnabled  False → True
  physics:approximation     convexHull → convexDecomposition  (L2_Left_Wall1_1 만)

실행:
  conda activate isaacsim
  python CoACD_CollisionFreeOff.py
  (Isaac Sim Script Editor 에서도 동작)
"""

import os
from pxr import Usd, UsdPhysics

USD_PATH = r"C:\Crusher_isaacsim\Crusher_IsaacSim_description\Crushing_CollisionFree.usd"

# ── 대상 정의 ────────────────────────────────────────────────────────────────
#   (collision_xform_path, approximation)
TARGETS = [
    (
        "/World/Crusher_IsaacSim"
        "/L2_Left_Wall1_1/collisions/L2_Left_Wall1_1/node_STL_BINARY_",
        "convexDecomposition",   # CoACD
    ),
    (
        "/World/Crusher_IsaacSim"
        "/L2_Wall3_1/collisions/L2_Wall3_1/node_STL_BINARY_",
        "convexHull",            # 단순 Collision
    ),
]

# ── Stage 열기 ───────────────────────────────────────────────────────────────
try:
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    SAVE_ON_EXIT = False   # Script Editor: 저장은 수동으로
    print("[모드] Isaac Sim Script Editor")
except ImportError:
    stage = Usd.Stage.Open(USD_PATH)
    SAVE_ON_EXIT = True
    print("[모드] 독립 실행 (저장 후 종료)")

assert stage, f"USD 파일을 열 수 없습니다: {USD_PATH}"

# ── 적용 ────────────────────────────────────────────────────────────────────
print()
for col_path, approx in TARGETS:
    prim = stage.GetPrimAtPath(col_path)
    assert prim.IsValid(), f"프림을 찾을 수 없습니다: {col_path}"

    # collisionEnabled: False → True
    prev_enabled = prim.GetAttribute("physics:collisionEnabled").Get()
    prim.GetAttribute("physics:collisionEnabled").Set(True)

    # approximation 변경
    prev_approx = prim.GetAttribute("physics:approximation").Get()
    prim.GetAttribute("physics:approximation").Set(approx)

    link_name = col_path.split("/")[3]   # L2_Left_Wall1_1 or L2_Wall3_1
    print(f"[{link_name}]")
    print(f"  collisionEnabled : {prev_enabled} → True")
    print(f"  approximation    : {prev_approx}  → {approx}")

# ── 저장 ────────────────────────────────────────────────────────────────────
if SAVE_ON_EXIT:
    stage.GetRootLayer().Save()
    print(f"\n저장 완료: {USD_PATH}")
else:
    print("\n※ Script Editor 모드: File > Save 로 직접 저장하세요.")

print("\nDone.")

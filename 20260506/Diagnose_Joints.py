"""
Diagnose_Joints.py - Motor2_Shaft 회전 불가 원인 추가 진단
"""

import omni.usd
from pxr import UsdPhysics, UsdGeom

stage = omni.usd.get_context().get_stage()

JOINT_PATH  = "/World/Crusher_IsaacSim/joints/L3_Motor2_1_L4_Motor2_Shaft_1"
CHILD_PATH  = "/World/Crusher_IsaacSim/L4_Motor2_Shaft_1"
ROBOT_PATH  = "/World/Crusher_IsaacSim"

print("=" * 50)

# 1. 조인트 활성화 여부
joint = stage.GetPrimAtPath(JOINT_PATH)
enabled = joint.GetAttribute("physics:jointEnabled").Get()
body0   = joint.GetAttribute("physics:body0").Get()
body1   = joint.GetAttribute("physics:body1").Get()
print(f"[joint] enabled={enabled}  body0={body0}  body1={body1}")

# 2. 자식 링크 RigidBody 여부
child = stage.GetPrimAtPath(CHILD_PATH)
has_rb  = child.HasAPI(UsdPhysics.RigidBodyAPI)
if has_rb:
    rb = UsdPhysics.RigidBodyAPI(child)
    kinematic = rb.GetKinematicEnabledAttr().Get()
    print(f"[child] RigidBodyAPI=True  kinematic={kinematic}")
else:
    print(f"[child] RigidBodyAPI=False  ← 이게 원인일 수 있음")

# 3. base_link 고정 여부
base = stage.GetPrimAtPath(f"{ROBOT_PATH}/base_link")
has_rb_base = base.HasAPI(UsdPhysics.RigidBodyAPI)
print(f"[base_link] RigidBodyAPI={has_rb_base}")
for prim in stage.Traverse():
    if prim.GetTypeName() == "PhysicsFixedJoint":
        b0 = prim.GetAttribute("physics:body0").Get()
        b1 = prim.GetAttribute("physics:body1").Get()
        if b0 is None or "base_link" in str(b1) or "base_link" in str(b0):
            print(f"[fixed-to-world] {prim.GetPath()}  body0={b0}  body1={b1}")

print("=" * 50)

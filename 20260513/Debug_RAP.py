"""
Debug_RAP.py  -  RackAndPinion 실패 원인 진단
Play 상태에서 실행
"""
import omni.usd
from pxr import UsdPhysics, PhysxSchema

ROBOT_PATH   = "/World/Crusher_IsaacSim"
RACK_JOINT   = f"{ROBOT_PATH}/joints/L1_Guide1_1_L2_Left_Wall1_1"
PINION_JOINT = f"{ROBOT_PATH}/joints/L3_Motor2_1_L4_Motor2_Shaft_1"
PINION_BODY  = f"{ROBOT_PATH}/L4_Motor2_Shaft_1"
RACK_BODY    = f"{ROBOT_PATH}/L2_Left_Wall1_1"
RAP_PATH     = f"{ROBOT_PATH}/joints/RackAndPinionConstraint"

stage = omni.usd.get_context().get_stage()

# ── 1. 기본 prim 유효성 ──────────────────────────────────────────────────────
for label, path in [
    ("rack_joint",   RACK_JOINT),
    ("pinion_joint", PINION_JOINT),
    ("pinion_body",  PINION_BODY),
    ("rack_body",    RACK_BODY),
]:
    p = stage.GetPrimAtPath(path)
    print(f"[{label}]  valid={p.IsValid()}  type={p.GetTypeName() if p.IsValid() else 'N/A'}")

# ── 2. MimicJoint 속성 존재 여부 ─────────────────────────────────────────────
rack_prim = stage.GetPrimAtPath(RACK_JOINT)
if rack_prim.IsValid():
    for attr_name in [
        "physxMimicJoint:linear:gearing",
        "drive:linear:physics:stiffness",
        "drive:linear:physics:damping",
        "drive:linear:physics:maxForce",
    ]:
        a = rack_prim.GetAttribute(attr_name)
        print(f"  rack attr  {attr_name:<45}  valid={a.IsValid()}  val={a.Get() if a.IsValid() else 'N/A'}")

# ── 3. RAP prim ──────────────────────────────────────────────────────────────
rap_prim = stage.GetPrimAtPath(RAP_PATH)
print(f"\n[RAP]  valid={rap_prim.IsValid()}")

# ── 4. PhysxSchema 에 PhysxPhysicsRackAndPinionJoint 있는지 확인 ──────────────
has_api = hasattr(PhysxSchema, "PhysxPhysicsRackAndPinionJoint")
print(f"[PhysxSchema.PhysxPhysicsRackAndPinionJoint]  available={has_api}")

# ── 5. joints xform 존재 확인 ────────────────────────────────────────────────
joints_prim = stage.GetPrimAtPath(f"{ROBOT_PATH}/joints")
print(f"[joints xform]  valid={joints_prim.IsValid()}")

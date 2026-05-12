"""
Diagnose_Motor2.py - Motor2 joint 드라이브 설정 확인
Play 불필요, Script Editor에서 실행
"""
import omni.usd

stage = omni.usd.get_context().get_stage()
JOINT_PATH = "/World/Crusher_IsaacSim/joints/L3_Motor2_1_L4_Motor2_Shaft_1"

joint = stage.GetPrimAtPath(JOINT_PATH)
print(f"Applied schemas: {joint.GetAppliedSchemas()}")
for attr in joint.GetAttributes():
    v = attr.Get()
    if v is not None:
        print(f"  {attr.GetName()} = {v}")

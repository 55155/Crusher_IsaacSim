"""
Cleanup_MimicJoint.py
=====================
USD에 남아있는 모든 MimicJoint 속성을 직접 삭제하고 저장합니다.

Stop (■) 상태에서 실행하세요.
"""
import omni.usd

ROBOT_PATH   = "/World/Crusher_IsaacSim"
PINION_JOINT = f"{ROBOT_PATH}/joints/L3_Motor2_1_L4_Motor2_Shaft_1"
RACK_JOINT   = f"{ROBOT_PATH}/joints/L1_Guide1_1_L2_Left_Wall1_1"

stage  = omni.usd.get_context().get_stage()

removed_total = 0

for path in [PINION_JOINT, RACK_JOINT]:
    prim = stage.GetPrimAtPath(path)
    assert prim.IsValid(), f"없음: {path}"

    targets = [a.GetName() for a in prim.GetAttributes()
               if "mimic" in a.GetName().lower()]

    if targets:
        for name in targets:
            prim.RemoveProperty(name)
            print(f"  [삭제] {path.split('/')[-1]}  →  {name}")
            removed_total += 1
    else:
        print(f"  [없음] {path.split('/')[-1]}  mimic 속성 없음")

print(f"\n총 {removed_total}개 속성 삭제 완료")

stage.GetRootLayer().Save()
print("USD 저장 완료")
print("\n→ 이제 Play (▶) 후 Drive_Motor2_ConstVel.py 로 피니언 동작 확인")

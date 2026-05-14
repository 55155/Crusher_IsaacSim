"""
Cleanup_RAP.py  -  Stop 상태에서 실행
이전 스크립트가 USD에 저장한 RackAndPinionJoint prim 을 모두 제거합니다.
"""
import omni.usd

stage = omni.usd.get_context().get_stage()

REMOVE_PATHS = [
    "/World/RackAndPinionConstraint",
    "/World/Crusher_IsaacSim/joints/RackAndPinionConstraint",
]

for path in REMOVE_PATHS:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        stage.RemovePrim(path)
        print(f"[삭제] {path}")
    else:
        print(f"[없음] {path}")

stage.GetRootLayer().Save()
print("\nUSD 저장 완료. 이제 Play 후 RackAndPinion.py 를 실행하세요.")

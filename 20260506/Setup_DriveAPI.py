"""
Setup_DriveAPI.py  -  Isaac Sim Script Editor  (1회만 실행)

Crushing_CollisionFree.usd 내 모든 RevoluteJoint에 DriveAPI를 추가하고 저장.
실행 후에는 Drive_Motor2.py 단순 버전으로 제어 가능.

How to run:
  1. Open Crushing_CollisionFree.usd in Isaac Sim
  2. Run this script  (Play 불필요)
"""

import omni.usd
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()

added = []
for prim in stage.Traverse():
    if prim.GetTypeName() != "PhysicsRevoluteJoint":
        continue
    if prim.HasAPI(UsdPhysics.DriveAPI):
        print(f"  skip (already has DriveAPI): {prim.GetPath()}")
        continue

    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.GetTypeAttr().Set("force")
    drive.GetStiffnessAttr().Set(0.0)
    drive.GetDampingAttr().Set(0.0)
    drive.GetMaxForceAttr().Set(0.0)
    drive.GetTargetVelocityAttr().Set(0.0)
    added.append(str(prim.GetPath()))
    print(f"  DriveAPI added: {prim.GetPath()}")

stage.GetRootLayer().Save()
print(f"\nDone. {len(added)} joint(s) updated. USD saved.")
print("이제 Drive_Motor2.py 단순 버전으로 제어 가능합니다.")

"""
Diagnose_Units.py - Stage 단위 + Rack Drive 설정 확인
Script Editor에서 실행 (Play 불필요)
"""
import omni.usd
from pxr import UsdGeom, UsdPhysics

stage = omni.usd.get_context().get_stage()

# 1. Stage 단위
mpu = UsdGeom.GetStageMetersPerUnit(stage)
print(f"metersPerUnit = {mpu}  →  1 stage-unit = {mpu*100:.1f} cm  ({mpu*1000:.1f} mm)")

# 2. Rack drive 설정
RACK_PATH = "/World/Crusher_IsaacSim/joints/L1_Guide1_1_L2_Left_Wall1_1"
rack = stage.GetPrimAtPath(RACK_PATH)
print(f"\nRack applied schemas: {rack.GetAppliedSchemas()}")
for attr in rack.GetAttributes():
    v = attr.Get()
    if v is not None:
        print(f"  {attr.GetName()} = {v}")

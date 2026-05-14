"""
Diag_RemoveMimic.py
===================
MimicJoint 를 모두 제거하고 피니언 Drive 단독 동작 확인.

Stop 상태에서 실행 → USD 저장 → Play → 결과 확인
"""
import omni.usd, omni.physx, omni.timeline
from pxr import UsdPhysics, PhysxSchema, Sdf

ROBOT_PATH   = "/World/Crusher_IsaacSim"
PINION_JOINT = f"{ROBOT_PATH}/joints/L3_Motor2_1_L4_Motor2_Shaft_1"
RACK_JOINT   = f"{ROBOT_PATH}/joints/L1_Guide1_1_L2_Left_Wall1_1"

stage   = omni.usd.get_context().get_stage()
r_prim  = stage.GetPrimAtPath(RACK_JOINT)
p_prim  = stage.GetPrimAtPath(PINION_JOINT)

# ── 1. 랙에서 MimicJointAPI 관련 속성 전부 제거 ──────────────────────────────
print("=== 제거 전 Mimic 속성 ===")
mimic_attrs = [a for a in r_prim.GetAttributes() if "mimic" in a.GetName().lower()]
for a in mimic_attrs:
    print(f"  {a.GetName()} = {a.Get()}")

# RemoveAPI 호출로 MimicJoint 완전 제거
removed = []
for token in [UsdPhysics.Tokens.rotX, UsdPhysics.Tokens.rotY, UsdPhysics.Tokens.rotZ,
              UsdPhysics.Tokens.transX, UsdPhysics.Tokens.transY, UsdPhysics.Tokens.transZ]:
    try:
        if PhysxSchema.PhysxMimicJointAPI(r_prim, token):
            PhysxSchema.PhysxMimicJointAPI.CanApply(r_prim, token)  # check
            r_prim.RemoveAPI(PhysxSchema.PhysxMimicJointAPI, token)
            removed.append(str(token))
    except:
        pass

# "linear" 토큰으로 적용된 경우도 제거 (URDF 변환 잔재)
for attr in list(r_prim.GetAttributes()):
    if "mimic" in attr.GetName().lower():
        r_prim.RemoveProperty(attr.GetName())

print(f"\n제거된 MimicJoint 토큰: {removed if removed else '없음 (속성 직접 삭제)'}")

# ── 2. 피니언 limit 확인 ──────────────────────────────────────────────────────
print("\n=== 조인트 정보 ===")
for label, prim in [("PINION", p_prim), ("RACK", r_prim)]:
    axis = prim.GetAttribute("physics:axis").Get()
    lo   = prim.GetAttribute("physics:lowerLimit").Get()
    hi   = prim.GetAttribute("physics:upperLimit").Get()
    print(f"  {label:8s}  axis={axis}  limit=({lo}, {hi})")

# ── 3. 피니언 Drive 설정 ──────────────────────────────────────────────────────
drive = UsdPhysics.DriveAPI.Get(p_prim, "angular")
drive.GetStiffnessAttr().Set(0.0)
drive.GetDampingAttr().Set(1e6)
drive.GetMaxForceAttr().Set(1e9)
drive.GetTargetVelocityAttr().Set(0.0)   # Play 후 start() 로 설정
print("\n[Drive] 설정 완료 (damping=1e6, maxForce=1e9)")

# ── 4. 랙 Drive 수동화 ────────────────────────────────────────────────────────
for name in ["drive:linear:physics:stiffness",
             "drive:linear:physics:damping",
             "drive:linear:physics:maxForce"]:
    a = r_prim.GetAttribute(name)
    if a.IsValid():
        a.Set(0.0)
print("[Rack Drive] passive")

# ── 5. USD 저장 ───────────────────────────────────────────────────────────────
stage.GetRootLayer().Save()
print("\n[USD 저장 완료]\n")
print("→ Play (▶) 누르고 아래 입력:")
print("   start(10)   ← Pinion 10 rad/s")

# ── 콜백 ─────────────────────────────────────────────────────────────────────
_target = 0.0
_sub    = None
_cnt    = 0
p_state = PhysxSchema.JointStateAPI.Apply(p_prim, UsdPhysics.Tokens.angular)
r_state = PhysxSchema.JointStateAPI.Apply(r_prim, UsdPhysics.Tokens.linear)

def _on_step(dt):
    global _cnt
    drive.GetTargetVelocityAttr().Set(_target)
    _cnt += 1
    if _cnt % 60 != 0:
        return
    t  = _cnt // 60
    pv = p_state.GetVelocityAttr().Get() or 0.0
    pp = p_state.GetPositionAttr().Get() or 0.0
    rv = r_state.GetVelocityAttr().Get() or 0.0
    rp = r_state.GetPositionAttr().Get() or 0.0
    print(f"  [t={t:>2}s]  Pinion {float(pv):+7.2f} deg/s  pos={float(pp):+7.2f} deg"
          f"  |  Rack {float(rv)*1000:+7.3f} mm/s  pos={float(rp)*1000:+7.3f} mm")

def start(vel_rad=10.0):
    global _target
    _target = vel_rad * 180.0 / 3.14159  # rad/s → deg/s (DriveAPI 단위)
    print(f"[start] {vel_rad} rad/s ({_target:.1f} deg/s)")

def stop():
    global _target
    _target = 0.0

_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_step)
omni.timeline.get_timeline_interface().set_end_time(86400.0)

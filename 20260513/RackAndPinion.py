"""
RackAndPinion.py  -  Isaac Sim Script Editor
=============================================
MimicJoint 방향:
  RACK 에 적용  (reference = PINION)
  "PINION 이 돌면 RACK 이 따라온다"

  gearing = -(r * π) / 180  [m/deg]
           = -(0.0175 * π) / 180 ≈ -0.000305  m/deg

실행:
  1. Stop (■)  →  Run  →  USD 저장
  2. Play (▶)  →  start(vel)

함수:
  start(vel)   Pinion vel rad/s  (기본 0.5)
  stop()
"""
import math
import omni.usd, omni.physx, omni.timeline
from pxr import UsdPhysics, PhysxSchema

ROBOT_PATH   = "/World/Crusher_IsaacSim"
PINION_JOINT = f"{ROBOT_PATH}/joints/L3_Motor2_1_L4_Motor2_Shaft_1"
RACK_JOINT   = f"{ROBOT_PATH}/joints/L1_Guide1_1_L2_Left_Wall1_1"

_RADIUS  = (1 * 35 / 2) / 1000             # 0.0175 m
GEARING  = -(_RADIUS * math.pi) / 180.0    # m/deg ≈ -0.000305

stage  = omni.usd.get_context().get_stage()
p_prim = stage.GetPrimAtPath(PINION_JOINT)
r_prim = stage.GetPrimAtPath(RACK_JOINT)
assert p_prim.IsValid() and r_prim.IsValid()

# ── 1. 기존 MimicJoint 잔재 완전 정리 (양쪽 모두) ─────────────────────────────
for prim in [p_prim, r_prim]:
    for attr in list(prim.GetAttributes()):
        if "mimic" in attr.GetName().lower():
            prim.RemoveProperty(attr.GetName())
print("[1] 기존 MimicJoint 속성 제거")

# ── 2. Pinion limit 설정 (MimicJoint 참조 조인트 필수 조건) ───────────────────
rev = UsdPhysics.RevoluteJoint(p_prim)
rev.GetLowerLimitAttr().Set(-1.0e10)
rev.GetUpperLimitAttr().Set( 1.0e10)
print("[2] Pinion limit → (-1e10, +1e10) deg")

# ── 3. MimicJoint → RACK 에 적용  (reference = PINION) ────────────────────────
mimic = PhysxSchema.PhysxMimicJointAPI.Apply(r_prim, UsdPhysics.Tokens.transX)
mimic.GetReferenceJointRel().SetTargets([PINION_JOINT])
mimic.GetReferenceJointAxisAttr().Set(UsdPhysics.Tokens.rotX)
mimic.GetGearingAttr().Set(GEARING)
mimic.GetOffsetAttr().Set(0.0)
print(f"[3] MimicJoint → RACK  (reference = PINION)")
print(f"    gearing = {GEARING:.8f} m/deg")

# ── 4. Rack drive → 완전 수동 ─────────────────────────────────────────────────
for name in ["drive:linear:physics:stiffness",
             "drive:linear:physics:damping",
             "drive:linear:physics:maxForce"]:
    a = r_prim.GetAttribute(name)
    if a.IsValid():
        a.Set(0.0)
print("[4] Rack drive → passive")

# ── 5. Pinion drive (velocity 모드) ───────────────────────────────────────────
drive = UsdPhysics.DriveAPI.Get(p_prim, "angular")
assert drive, "Pinion DriveAPI 없음"
drive.GetStiffnessAttr().Set(0.0)
drive.GetDampingAttr().Set(1e5)
drive.GetMaxForceAttr().Set(1e6)
drive.GetTargetVelocityAttr().Set(0.0)
print("[5] Pinion drive → damping=1e5  maxForce=1e6")

# ── 6. JointStateAPI ──────────────────────────────────────────────────────────
p_state = PhysxSchema.JointStateAPI.Apply(p_prim, UsdPhysics.Tokens.angular)
r_state = PhysxSchema.JointStateAPI.Apply(r_prim, UsdPhysics.Tokens.linear)

# ── 7. USD 저장 ───────────────────────────────────────────────────────────────
stage.GetRootLayer().Save()
print("[7] USD 저장 완료")
print("\n→ Play (▶) 후  start(vel)\n")

# ── 콜백 ──────────────────────────────────────────────────────────────────────
_target = 0.0
_sub    = None
_cnt    = 0


def _on_step(dt):
    global _cnt
    drive.GetTargetVelocityAttr().Set(_target * 180.0 / math.pi)  # rad/s → deg/s
    _cnt += 1
    if _cnt % 60 != 0:
        return

    pv = float(p_state.GetVelocityAttr().Get() or 0.0)   # deg/s
    pp = float(p_state.GetPositionAttr().Get() or 0.0)   # deg
    rv = float(r_state.GetVelocityAttr().Get() or 0.0)   # m/s
    rp = float(r_state.GetPositionAttr().Get() or 0.0)   # m

    print(f"  [t={_cnt//60:>3}s]"
          f"  Pinion {pv*math.pi/180:+.3f} rad/s  {pp:+7.2f} deg"
          f"  |  Rack {rv*1000:+7.3f} mm/s  {rp*1000:+8.3f} mm"
          f"  |  예상 {_target*_RADIUS*1000:+.3f} mm/s")


def start(vel: float = 0.5):
    global _target
    _target = vel
    print(f"[start]  Pinion {vel:+.3f} rad/s  →  Rack {vel*_RADIUS*1000:+.4f} mm/s 예상")


def stop():
    global _target
    _target = 0.0
    drive.GetTargetVelocityAttr().Set(0.0)
    print("[stop]")


_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_step)
omni.timeline.get_timeline_interface().set_end_time(86400.0)
print("콜백 등록 완료.  Play 후  start(vel)\n")

"""
Drive_Motor2_ConstVel.py  -  Isaac Sim Script Editor
=====================================================
L3_Motor2_1_L4_Motor2_Shaft_1 를 등속 회전시킵니다.

실행 순서:
  1. Isaac Sim 에서 Crushing_CollisionFree.usd 열기
  2. Play (▶) 누르기
  3. Script Editor 에 붙여넣고 [Run]

함수:
  start(vel)   vel rad/s 로 회전 시작  (기본 10 rad/s)
  stop()       정지
"""

import math
import omni.usd
import omni.timeline
import omni.physx
from pxr import UsdPhysics, PhysxSchema

JOINT_PATH = "/World/Crusher_IsaacSim/joints/L3_Motor2_1_L4_Motor2_Shaft_1"
DAMPING    = 1e5     # N·m·s/deg
MAX_FORCE  = 1e6     # N·m

# ── USD 조인트 prim ──────────────────────────────────────────────────────────
_stage = omni.usd.get_context().get_stage()
_joint = _stage.GetPrimAtPath(JOINT_PATH)
assert _joint.IsValid(), f"조인트를 찾을 수 없습니다:\n  {JOINT_PATH}"

_drive = UsdPhysics.DriveAPI.Get(_joint, "angular")
assert _drive, "DriveAPI 가 없습니다. Setup_DriveAPI.py 를 먼저 실행하세요."

# ── 조인트 limit 해제 (회전 범위를 막지 않도록) ────────────────────────────────
_rev = UsdPhysics.RevoluteJoint(_joint)
_lo  = _rev.GetLowerLimitAttr().Get()
_hi  = _rev.GetUpperLimitAttr().Get()
print(f"[Motor2] 기존 limit: lower={_lo}  upper={_hi}")
_rev.GetLowerLimitAttr().Set(-1.0e10)
_rev.GetUpperLimitAttr().Set( 1.0e10)
print("[Motor2] limit → (-1e10, +1e10) deg  ← 무제한 회전")


_target_deg = 0.0
_sub        = None
_cnt        = 0

_p_state = PhysxSchema.JointStateAPI.Apply(_joint, UsdPhysics.Tokens.angular)


def _on_step(dt):
    global _cnt
    _drive.GetTargetVelocityAttr().Set(_target_deg)
    _cnt += 1
    if _cnt % 60 != 0:
        return
    pv = float(_p_state.GetVelocityAttr().Get() or 0.0)   # deg/s
    pp = float(_p_state.GetPositionAttr().Get() or 0.0)   # deg
    print(f"  [t={_cnt//60:>3}s]  Pinion {pv*math.pi/180:+.3f} rad/s  {pp:+8.2f} deg")


def start(vel: float = 10.0):
    """vel rad/s 로 등속 회전."""
    global _target_deg
    _target_deg = vel * 180.0 / math.pi
    _drive.GetStiffnessAttr().Set(0.0)
    _drive.GetDampingAttr().Set(DAMPING)
    _drive.GetMaxForceAttr().Set(MAX_FORCE)
    print(f"[Motor2] start  →  {vel:+.2f} rad/s  ({_target_deg:+.1f} deg/s)")


def stop():
    global _target_deg
    _target_deg = 0.0
    print("[Motor2] stop")


omni.timeline.get_timeline_interface().set_end_time(86400.0)
_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_step)
print("[Motor2] 콜백 등록 완료")
start()

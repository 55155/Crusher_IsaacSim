"""
Drive_PrismaticJoint.py  -  Isaac Sim Script Editor
====================================================
L1_Guide1_1_L2_Left_Wall1_1 (Prismatic / X축) 를 등속으로 구동합니다.

실행 순서:
  1. Isaac Sim 에서 Crushing_CollisionFree.usd 열기
  2. Play (▶) 누르기
  3. Script Editor 에 붙여넣고 [Run]

함수:
  start(vel)   vel m/s 로 이동 시작  (기본 0.01 m/s = 10 mm/s)
  stop()       정지
"""

import omni.usd
import omni.timeline
from pxr import UsdPhysics

JOINT_PATH = "/World/Crusher_IsaacSim/joints/L1_Guide1_1_L2_Left_Wall1_1"
DAMPING    = 10000.0   # N·s/m  — USD 기존값 유지
MAX_FORCE  = 10000.0   # N      — USD 기존값 유지

# ── USD 조인트 prim ──────────────────────────────────────────────────────────
_stage = omni.usd.get_context().get_stage()
_joint = _stage.GetPrimAtPath(JOINT_PATH)
assert _joint.IsValid(), f"조인트를 찾을 수 없습니다:\n  {JOINT_PATH}"

_drive = UsdPhysics.DriveAPI.Get(_joint, "linear")   # Prismatic → "linear"
assert _drive, "DriveAPI(linear) 가 없습니다."


def start(vel: float = -0.01):
    """vel m/s 로 등속 이동.  음수 → 반대 방향."""
    _drive.GetStiffnessAttr().Set(0.0)
    _drive.GetDampingAttr().Set(DAMPING)
    _drive.GetMaxForceAttr().Set(MAX_FORCE)
    _drive.GetTargetVelocityAttr().Set(vel)
    print(f"[Prismatic] start  →  {vel*1000:+.1f} mm/s")
    print(f"            범위 제한: -500 ~ +500 mm  (한쪽 끝에 닿으면 자동 정지)")


def stop():
    """속도 0 으로 설정."""
    _drive.GetTargetVelocityAttr().Set(0.0)
    print("[Prismatic] stop")


# ── 타임라인 종료 시간 연장 ──────────────────────────────────────────────────
omni.timeline.get_timeline_interface().set_end_time(86400.0)

# ── 스크립트 실행 시 자동 시작 ─────────────────────────────────────────────
start()

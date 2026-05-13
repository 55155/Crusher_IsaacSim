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

import omni.usd
import omni.timeline
from pxr import UsdPhysics

JOINT_PATH = "/World/Crusher_IsaacSim/joints/L3_Motor2_1_L4_Motor2_Shaft_1"
DAMPING    = 200.0   # N·m·s/rad  — 속도 추종력
MAX_FORCE  = 100.0   # N·m        — USD 기존값 유지

# ── USD 조인트 prim ──────────────────────────────────────────────────────────
_stage = omni.usd.get_context().get_stage()
_joint = _stage.GetPrimAtPath(JOINT_PATH)
assert _joint.IsValid(), f"조인트를 찾을 수 없습니다:\n  {JOINT_PATH}"

_drive = UsdPhysics.DriveAPI.Get(_joint, "angular")
assert _drive, "DriveAPI 가 없습니다. Setup_DriveAPI.py 를 먼저 실행하세요."


def start(vel: float = 10.0):
    """vel rad/s 로 등속 회전."""
    _drive.GetStiffnessAttr().Set(0.0)      # position 추종 비활성
    _drive.GetDampingAttr().Set(DAMPING)
    _drive.GetMaxForceAttr().Set(MAX_FORCE)
    _drive.GetTargetVelocityAttr().Set(vel)
    print(f"[Motor2] start  →  {vel:+.2f} rad/s  "
          f"(damping={DAMPING}, maxForce={MAX_FORCE})")


def stop():
    """속도 0 으로 설정."""
    _drive.GetTargetVelocityAttr().Set(0.0)
    print("[Motor2] stop")


# ── 타임라인 종료 시간 연장 (기본값이 짧아 자동 정지되는 문제 방지) ──────────
omni.timeline.get_timeline_interface().set_end_time(86400.0)  # 24시간 (초)
print("[Motor2] 타임라인 종료 시간 → 24시간으로 연장")

# ── 스크립트 실행 시 자동 시작 ─────────────────────────────────────────────
start()

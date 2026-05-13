"""
RackAndPinion.py  -  Isaac Sim Script Editor
=============================================
Pinion(Revolute) 을 구동하면 MimicJoint 를 통해 Rack(Prismatic) 이 자동 추종합니다.

  기어 사양  : 모듈=1, 이빨=35
  피치 반경  : r = module × teeth / 2 = 1 × 35 / 2 = 17.5 mm = 0.0175 m
  기어링     : rack_displacement = -0.0175 × pinion_angle  [m/rad]
  (음수 → Pinion +Z 회전 시 Rack -X 방향 이동)

실행 순서:
  1. Isaac Sim 에서 Crushing_CollisionFree.usd 열기
  2. Play (▶) 누르기
  3. Script Editor 에 붙여넣고 [Run]

함수:
  start(vel)  Pinion vel rad/s 로 구동  (기본 10 rad/s)
  stop()      정지
"""

import omni.usd
import omni.physx
import omni.timeline
from pxr import UsdPhysics

# ── 경로 ────────────────────────────────────────────────────────────────────
ROBOT_PATH  = "/World/Crusher_IsaacSim"
PINION_PATH = f"{ROBOT_PATH}/joints/L3_Motor2_1_L4_Motor2_Shaft_1"
RACK_PATH   = f"{ROBOT_PATH}/joints/L1_Guide1_1_L2_Left_Wall1_1"
PINION_NAME = "L3_Motor2_1_L4_Motor2_Shaft_1"   # DOF 이름 검색용

# ── 기어 사양 ────────────────────────────────────────────────────────────────
_MODULE = 1        # mm
_TEETH  = 35
_RADIUS = _MODULE * _TEETH / 2 / 1000   # m  →  0.0175
GEARING = -_RADIUS                       # m/rad  →  -0.0175

# ── Pinion Drive 파라미터 ────────────────────────────────────────────────────
PINION_DAMPING  = 200.0    # N·m·s/rad
PINION_MAXFORCE = 100.0    # N·m

# ── USD 설정 (Play 전/후 모두 가능) ─────────────────────────────────────────
stage = omni.usd.get_context().get_stage()

# 1) MimicJoint gearing 확인 및 재설정
rack = stage.GetPrimAtPath(RACK_PATH)
assert rack.IsValid(), f"Rack 조인트를 찾을 수 없습니다: {RACK_PATH}"
rack.GetAttribute("physxMimicJoint:linear:gearing").Set(GEARING)
rack.GetAttribute("physxMimicJoint:linear:offset").Set(0.0)
print(f"MimicJoint  gearing = {GEARING:.6f} m/rad  (module={_MODULE}, teeth={_TEETH})")

# 2) Rack Drive → 완전 수동(passive): MimicJoint 가 단독으로 제어
#    damping > 0 이면 MimicJoint 의 움직임을 제동하므로 0 으로 설정
rack.GetAttribute("drive:linear:physics:stiffness").Set(0.0)
rack.GetAttribute("drive:linear:physics:damping").Set(0.0)
rack.GetAttribute("drive:linear:physics:maxForce").Set(0.0)
rack.GetAttribute("physxJoint:maxJointVelocity").Set(abs(GEARING) * 200)

# 3) Pinion Drive: velocity 모드 (stiffness=0)
pinion = stage.GetPrimAtPath(PINION_PATH)
assert pinion.IsValid(), f"Pinion 조인트를 찾을 수 없습니다: {PINION_PATH}"
drive = UsdPhysics.DriveAPI.Get(pinion, "angular")
assert drive, "Pinion 에 DriveAPI 가 없습니다."
drive.GetStiffnessAttr().Set(0.0)
drive.GetDampingAttr().Set(PINION_DAMPING)
drive.GetMaxForceAttr().Set(PINION_MAXFORCE)

# ── 매 step Pinion targetVelocity 갱신 (USD DriveAPI 직접 제어) ──────────────
# Articulation 미사용: set_joint_velocity_targets 가 Rack DOF를 0으로 덮어쓰면
# MimicJoint 가 무력화되기 때문
_target   = 0.0
_sub      = None
_step_cnt = 0


def _on_step(dt):
    global _step_cnt
    # Pinion 속도 갱신
    drive.GetTargetVelocityAttr().Set(_target)

    # 매 60 step (~1초) 상태 출력
    _step_cnt += 1
    if _step_cnt % 60 != 0:
        return

    pin_vel  = pinion.GetAttribute("state:angular:physics:velocity").Get()
    pin_pos  = pinion.GetAttribute("state:angular:physics:position").Get()
    rack_vel = rack.GetAttribute("state:linear:physics:velocity").Get()
    rack_pos = rack.GetAttribute("state:linear:physics:position").Get()

    pin_vel  = pin_vel  if pin_vel  is not None else float("nan")
    pin_pos  = pin_pos  if pin_pos  is not None else float("nan")
    rack_vel = rack_vel if rack_vel is not None else float("nan")
    rack_pos = rack_pos if rack_pos is not None else float("nan")

    obs_gear = (rack_vel / pin_vel) if abs(pin_vel) > 1e-6 else float("nan")

    print(f"  [t={_step_cnt//60:>3}s]"
          f"  Pinion {pin_vel:+.4f} rad/s  {pin_pos:+7.4f} rad"
          f"  |  Rack {rack_vel*1000:+6.3f} mm/s  {rack_pos*1000:+7.3f} mm"
          f"  |  관측기어링 {obs_gear*1000:+.4f} mm/rad"
          f"  (설정 {GEARING*1000:.4f})")


def start(vel: float = 10.0):
    global _target
    _target = vel
    print(f"[RackAndPinion] Pinion {vel:+.1f} rad/s  →  Rack {vel*abs(GEARING)*1000:+.2f} mm/s 예상")


def stop():
    global _target
    _target = 0.0
    drive.GetTargetVelocityAttr().Set(0.0)
    print("[RackAndPinion] 정지")


# ── 콜백 등록 (전역 보관 → GC 방지) ─────────────────────────────────────────
_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_step)
omni.timeline.get_timeline_interface().set_end_time(86400.0)

print(f"\n피치 반경 = {_RADIUS*1000:.1f} mm  |  gearing = {GEARING:.6f} m/rad")
print("콜백 등록 완료 → start() / stop() 으로 제어하세요.\n")
start(0.2)   # 극단적으로 느리게: Pinion 0.2 rad/s → Rack 3.5 mm/s

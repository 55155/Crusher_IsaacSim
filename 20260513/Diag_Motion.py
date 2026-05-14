"""
Diag_Motion.py  -  Play 상태에서 실행
1. 조인트 축 확인
2. 피니언 실제 회전 여부 확인
3. MimicJoint 현재 속성 덤프
"""
import math, omni.usd, omni.physx, omni.timeline
from pxr import UsdPhysics, PhysxSchema

ROBOT_PATH   = "/World/Crusher_IsaacSim"
PINION_JOINT = f"{ROBOT_PATH}/joints/L3_Motor2_1_L4_Motor2_Shaft_1"
RACK_JOINT   = f"{ROBOT_PATH}/joints/L1_Guide1_1_L2_Left_Wall1_1"

stage       = omni.usd.get_context().get_stage()
rack_prim   = stage.GetPrimAtPath(RACK_JOINT)
pinion_prim = stage.GetPrimAtPath(PINION_JOINT)

# ── 1. 조인트 축 ──────────────────────────────────────────────────────────────
print("=== 조인트 축 ===")
for label, prim in [("PINION (Revolute)", pinion_prim), ("RACK (Prismatic)", rack_prim)]:
    axis = prim.GetAttribute("physics:axis").Get()
    lower = prim.GetAttribute("physics:lowerLimit").Get()
    upper = prim.GetAttribute("physics:upperLimit").Get()
    print(f"  {label}  axis={axis}  limit=({lower}, {upper})")

# ── 2. MimicJoint 속성 전체 덤프 ─────────────────────────────────────────────
print("\n=== MimicJoint 속성 (rack prim) ===")
for attr in rack_prim.GetAttributes():
    if "mimic" in attr.GetName().lower():
        print(f"  {attr.GetName()} = {attr.Get()}")

# ── 3. DriveAPI 속성 ──────────────────────────────────────────────────────────
print("\n=== Rack Drive 속성 ===")
for attr in rack_prim.GetAttributes():
    if "drive" in attr.GetName().lower():
        print(f"  {attr.GetName()} = {attr.Get()}")

print("\n=== Pinion Drive 속성 ===")
for attr in pinion_prim.GetAttributes():
    if "drive" in attr.GetName().lower():
        print(f"  {attr.GetName()} = {attr.Get()}")

# ── 4. 피니언 실제 회전 모니터링 (10초간) ─────────────────────────────────────
print("\n=== 피니언 강제 구동 테스트 (10초) ===")
drive = UsdPhysics.DriveAPI.Get(pinion_prim, "angular")
drive.GetStiffnessAttr().Set(0.0)
drive.GetDampingAttr().Set(200.0)
drive.GetMaxForceAttr().Set(100.0)
drive.GetTargetVelocityAttr().Set(10.0)   # 10 rad/s 강제 명령

pinion_state = PhysxSchema.JointStateAPI.Apply(pinion_prim, UsdPhysics.Tokens.angular)
rack_state   = PhysxSchema.JointStateAPI.Apply(rack_prim,   UsdPhysics.Tokens.linear)

_cnt = [0]
_sub = [None]

def _on_step(dt):
    drive.GetTargetVelocityAttr().Set(10.0)
    _cnt[0] += 1
    if _cnt[0] % 60 != 0:
        return

    t = _cnt[0] // 60
    pv = pinion_state.GetVelocityAttr().Get()
    pp = pinion_state.GetPositionAttr().Get()
    rv = rack_state.GetVelocityAttr().Get()
    rp = rack_state.GetPositionAttr().Get()

    pv = float(pv) if pv is not None else float("nan")
    pp = float(pp) if pp is not None else float("nan")
    rv = float(rv) if rv is not None else float("nan")
    rp = float(rp) if rp is not None else float("nan")

    print(f"  [t={t}s]  Pinion {pv:+.2f} deg/s ({pv*math.pi/180:+.3f} rad/s)  pos={pp:+.2f} deg"
          f"  |  Rack {rv*1000:+.3f} mm/s  pos={rp*1000:+.3f} mm")

    if t >= 10:
        drive.GetTargetVelocityAttr().Set(0.0)
        print("\n[진단 완료]")
        if abs(pv) < 0.1:
            print("  ❌ 피니언이 회전하지 않음 → Drive 문제")
        elif abs(rv) < 0.001:
            print("  ❌ 피니언은 회전하지만 랙이 안 움직임 → MimicJoint 문제")
        else:
            print("  ✅ 양쪽 모두 작동 중")
        _sub[0] = None   # 구독 해제

_sub[0] = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_step)
omni.timeline.get_timeline_interface().set_end_time(86400.0)
print("10초간 Pinion 10 rad/s 강제 구동 중...")

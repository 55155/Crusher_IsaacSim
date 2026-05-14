"""
Diag_DriveOnly.py
=================
MimicJoint 완전 무시, 피니언 Drive 단독 테스트.
"피니언이 회전하는가?" 만 확인.

Stop → Play → 이 스크립트만 단독 실행
"""
import math, omni.usd, omni.physx, omni.timeline
from pxr import UsdPhysics, PhysxSchema

ROBOT_PATH   = "/World/Crusher_IsaacSim"
PINION_JOINT = f"{ROBOT_PATH}/joints/L3_Motor2_1_L4_Motor2_Shaft_1"
RACK_JOINT   = f"{ROBOT_PATH}/joints/L1_Guide1_1_L2_Left_Wall1_1"

stage     = omni.usd.get_context().get_stage()
p_prim    = stage.GetPrimAtPath(PINION_JOINT)
r_prim    = stage.GetPrimAtPath(RACK_JOINT)

# ── 조인트 정보 출력 ─────────────────────────────────────────────────────────
print("=== 조인트 축 ===")
for label, prim in [("PINION", p_prim), ("RACK", r_prim)]:
    axis  = prim.GetAttribute("physics:axis").Get()
    lo    = prim.GetAttribute("physics:lowerLimit").Get()
    hi    = prim.GetAttribute("physics:upperLimit").Get()
    print(f"  {label:8s}  axis={axis}  limit=({lo}, {hi})")

# ── Pinion Drive 강제 설정 (최대 힘으로) ─────────────────────────────────────
drive = UsdPhysics.DriveAPI.Get(p_prim, "angular")
assert drive, "Pinion DriveAPI 없음"
drive.GetStiffnessAttr().Set(0.0)
drive.GetDampingAttr().Set(1e6)    # 매우 강한 속도 추종
drive.GetMaxForceAttr().Set(1e9)   # 제한 없음에 가깝게
drive.GetTargetVelocityAttr().Set(10.0)   # 10 rad/s
print("\n[Drive] stiffness=0  damping=1e6  maxForce=1e9  targetVel=10 rad/s")

# ── JointStateAPI ─────────────────────────────────────────────────────────────
p_state = PhysxSchema.JointStateAPI.Apply(p_prim, UsdPhysics.Tokens.angular)
r_state = PhysxSchema.JointStateAPI.Apply(r_prim, UsdPhysics.Tokens.linear)

# ── 콜백 ─────────────────────────────────────────────────────────────────────
_cnt  = 0
_sub  = None

def _on_step(dt):
    global _cnt
    drive.GetTargetVelocityAttr().Set(10.0)   # 매 step 갱신
    _cnt += 1
    if _cnt % 60 != 0:
        return

    t  = _cnt // 60
    pv = p_state.GetVelocityAttr().Get()
    pp = p_state.GetPositionAttr().Get()
    rv = r_state.GetVelocityAttr().Get()
    rp = r_state.GetPositionAttr().Get()

    pv = float(pv) if pv is not None else float("nan")
    pp = float(pp) if pp is not None else float("nan")
    rv = float(rv) if rv is not None else float("nan")
    rp = float(rp) if rp is not None else float("nan")

    print(f"  [t={t:>2}s]  Pinion {pv:+8.3f} deg/s  pos={pp:+8.2f} deg"
          f"  |  Rack {rv*1000:+7.3f} mm/s  pos={rp*1000:+7.3f} mm")

    if t == 5:
        if abs(pv) < 0.1:
            print("\n  ❌ 피니언 미동작 → 조인트가 잠겨있거나 Drive 설정 문제")
        else:
            print(f"\n  ✅ 피니언 동작 확인: {pv:.1f} deg/s = {pv*math.pi/180:.3f} rad/s")
            if abs(rv) < 0.001:
                print("  ❌ 랙 미동작 → MimicJoint 문제")
            else:
                print(f"  ✅ 랙도 동작: {rv*1000:.3f} mm/s")

_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_step)
omni.timeline.get_timeline_interface().set_end_time(86400.0)
print("\n5초간 모니터링 시작...\n")

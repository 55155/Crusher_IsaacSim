"""
MimicJoint.py  -  Isaac Sim Script Editor
Motor2(Pinion) 구동  -  Rack은 physxMimicJoint로 자동 추종

  physxMimicJoint:linear:gearing = -0.0175  (USD에 이미 설정됨)
  → Pinion만 돌리면 Rack이 자동으로 비율에 맞게 이동

  문제였던 원인: physxJoint:maxJointVelocity = 0.005 m/s (하드캡)
  → maxJointVelocity와 maxForce를 올려줘야 Rack이 따라올 수 있음

How to run:
  1. Open Crushing_CollisionFree.usd
  2. Stop → Play
  3. Run this script
"""

import numpy as np
import omni.physx
import omni.usd

ROBOT_PATH = "/World/Crusher_IsaacSim"
RACK_PATH  = f"{ROBOT_PATH}/joints/L1_Guide1_1_L2_Left_Wall1_1"

PINION_DOF = 2      # L3_Motor2_1_L4_Motor2_Shaft_1
MOTOR_VEL  = 10.0   # rad/s

# Rack 물리 제한 해제 (maxJointVelocity 0.005 → 2.0 m/s, maxForce 5 → 1e4)
stage     = omni.usd.get_context().get_stage()
rack_prim = stage.GetPrimAtPath(RACK_PATH)
rack_prim.GetAttribute("physxJoint:maxJointVelocity").Set(2.0)
rack_prim.GetAttribute("drive:linear:physics:maxForce").Set(1e4)
rack_prim.GetAttribute("drive:linear:physics:stiffness").Set(0.0)
rack_prim.GetAttribute("drive:linear:physics:damping").Set(1e4)
rack_prim.GetAttribute("physics:lowerLimit").Set(-0.5)
rack_prim.GetAttribute("physics:upperLimit").Set( 0.5)
print(f"Rack 제한 해제: maxVel=2.0 m/s  maxForce=1e4  stiffness=0  damping=1e4")

try:
    from isaacsim.core.prims import ArticulationView
except ImportError:
    from omni.isaac.core.articulations import ArticulationView

_view  = ArticulationView(prim_paths_expr=ROBOT_PATH, name="crusher_rp")
_ready = False

def _on_step(dt):
    global _ready
    if not _ready:
        try:
            _view.initialize()
            _ready = _view.is_physics_handle_valid()
            if _ready:
                print(f"[OK] Pinion DOF[{PINION_DOF}] @ {MOTOR_VEL} rad/s  "
                      f"→ Rack 자동 추종 (gearing=-0.0175)")
        except Exception as e:
            print(f"init: {e}")
        return

    vel = np.zeros((1, _view.num_dof))
    vel[0, PINION_DOF] = MOTOR_VEL
    _view.set_joint_velocity_targets(velocities=vel)

_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_step)
print(f"시작: Pinion {MOTOR_VEL} rad/s → Rack {MOTOR_VEL * 0.0175 * 1000:.1f} mm/s 예상")

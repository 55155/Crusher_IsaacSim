"""
Drive_Motor2.py  -  Isaac Sim Script Editor

How to run:
  1. Open Crushing_CollisionFree.usd in Isaac Sim
  2. Press Play
  3. Run this script
"""

import numpy as np
import omni.physx
import omni.usd

try:
    from isaacsim.core.prims import ArticulationView
except ImportError:
    from omni.isaac.core.articulations import ArticulationView

ROBOT_PATH  = "/World/Crusher_IsaacSim"
JOINT_PATH  = f"{ROBOT_PATH}/joints/L3_Motor2_1_L4_Motor2_Shaft_1"
TARGET_VEL  = 20.0   # rad/s
DOF_IDX     = 2      # L3_Motor2_1_L4_Motor2_Shaft_1

# stiffness=200 + 누적된 targetPosition이 순간 폭발적 가속을 유발
# → 순수 velocity drive로 전환 (stiffness=0)
stage = omni.usd.get_context().get_stage()
joint = stage.GetPrimAtPath(JOINT_PATH)
joint.GetAttribute("drive:angular:physics:stiffness").Set(0.0)
joint.GetAttribute("drive:angular:physics:targetPosition").Set(0.0)
joint.GetAttribute("drive:angular:physics:damping").Set(100.0)
joint.GetAttribute("drive:angular:physics:maxForce").Set(1e4)
print("Drive 초기화: stiffness=0  damping=100  maxForce=1e4")

_view  = ArticulationView(prim_paths_expr=ROBOT_PATH, name="crusher_v")
_ready = False

def _on_step(dt):
    global _ready
    if not _ready:
        try:
            _view.initialize()
            _ready = _view.is_physics_handle_valid()
            if _ready:
                print(f"OK: DOF[{DOF_IDX}] @ {TARGET_VEL} rad/s")
        except Exception as e:
            print(f"init failed: {e}")
        return

    vel = np.zeros((1, _view.num_dof))
    vel[0, DOF_IDX] = TARGET_VEL
    _view.set_joint_velocity_targets(velocities=vel)

_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_step)
print("Callback registered. (Play 후 자동 시작)")

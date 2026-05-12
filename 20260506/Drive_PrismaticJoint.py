"""
Drive_PrismaticJoint.py  -  Isaac Sim Script Editor
-----------------------------------------------------
Moves L1_Guide1_1_L2_Left_Wall1_1 to -10 mm (negative direction).

How to run:
  1. Open Crushing_CollisionFree.usd in Isaac Sim
  2. Press Play
  3. Window > Script Editor -> paste this file -> Run
"""

import omni.usd
from pxr import UsdPhysics

JOINT_PATH = "/World/Crusher_IsaacSim/joints/L1_Guide1_1_L2_Left_Wall1_1"
TARGET_M   = -0.010   # -10 mm (negative direction)

stage = omni.usd.get_context().get_stage()
prim  = stage.GetPrimAtPath(JOINT_PATH)

# Allow travel in negative direction
prim.GetAttribute("physics:lowerLimit").Set(TARGET_M)
prim.GetAttribute("physics:upperLimit").Set(0.0)

# Drive parameters
prim.GetAttribute("drive:linear:physics:stiffness").Set(50.0)    # N/m  (low = slow approach)
prim.GetAttribute("drive:linear:physics:damping").Set(20.0)      # N*s/m (high = smooth)
prim.GetAttribute("drive:linear:physics:maxForce").Set(5.0)      # N
prim.GetAttribute("drive:linear:physics:type").Set("force")

# Velocity cap: 5 mm/s  ->  10 mm takes ~2 seconds
prim.GetAttribute("physxJoint:maxJointVelocity").Set(0.005)      # m/s

# Move to -10 mm
prim.GetAttribute("drive:linear:physics:targetPosition").Set(TARGET_M)

print("Target set to %.0f mm" % (TARGET_M * 1000))

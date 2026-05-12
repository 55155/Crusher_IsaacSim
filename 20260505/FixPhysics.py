"""
FixPhysics.py  -  Isaac Sim 4.5 / 5.x  Script Editor
======================================================
Run this script FIRST in Script Editor before Drive_SlideJoint.py.

Root causes of "Invalid PhysX transform detected":
  1. Arm joints (L4_Shaft_1 chain, L5/L6/L7 links) have stiffness ~12 N*m/rad
     and damping ~0.005 N*m*s/rad -> arm collapses under gravity and diverges.
  2. Bearing joints (L7_Holder_Bearing1/2) have stiffness ~0.001, damping ~5e-7
     -> essentially unconstrained, spin to infinity.
  3. Prismatic joint limit = [0, 0] in the original USD -> constraint conflict
     when Drive_SlideJoint.py tries to move it.

This script fixes all three issues without modifying the USD file on disk.

Usage in Script Editor:
  1. Load Crushing.usd
  2. Paste this script -> click [Run]
  3. Paste Drive_SlideJoint.py -> click [Run]
  4. Press Play
"""

import omni.usd
import omni.timeline
from pxr import UsdPhysics, Gf

# ==============================================================================
# Fix targets
# ==============================================================================

# Joints to LOCK (hold current position, not actuated during slide test)
# These either have no stiffness or would destabilize the arm.
#   format: (joint_path, stiffness [N*m/rad], damping [N*m*s/rad], max_force [N*m])
LOCK_JOINTS = [
    # Bevel gear shaft: free-spinning revolute (stiff=None, damp=0.5)
    ("/World/Crusher_IsaacSim/joints/L3_Bevel_GearBox_1_L4_Shaft_1",       2000.0, 200.0, 50.0),
    # Motor 2 shaft: free-spinning revolute (stiff=None, damp=0.5)
    ("/World/Crusher_IsaacSim/joints/L3_Motor2_1_L4_Motor2_Shaft_1",        2000.0, 200.0, 50.0),
    # Arm joint 1: stiffness=12.76, damping=0.005 -> collapse under gravity
    ("/World/Crusher_IsaacSim/joints/L5_Link1_1_L6_Link2_1",               3000.0, 300.0, 100.0),
    # Arm joint 2: stiffness=10.13, damping=0.004 -> collapse under gravity
    ("/World/Crusher_IsaacSim/joints/L6_Link2_1_L7_Link3_1",               3000.0, 300.0, 100.0),
    # Bearing 1: stiffness~0.001, damping~5e-7 -> free spin
    ("/World/Crusher_IsaacSim/joints/L6_DcutShaft_1_L7_Holder_Bearing1_1",  500.0,  50.0, 10.0),
    # Bearing 2: same as above
    ("/World/Crusher_IsaacSim/joints/L6_DcutShaft_1_L7_Holder_Bearing2_1",  500.0,  50.0, 10.0),
]

# Prismatic joint we WILL drive - just fix the limits here
SLIDE_JOINT_PATH = "/World/Crusher_IsaacSim/joints/L1_Guide1_1_L2_Left_Wall1_1"
SLIDE_STROKE_MIN =  0.000   # [m]
SLIDE_STROKE_MAX =  0.010   # [m]  (matches STROKE_MAX_M in Drive_SlideJoint.py)

# ==============================================================================
# Helpers
# ==============================================================================

def _stage():
    return omni.usd.get_context().get_stage()


def _attr_set(prim, name, value):
    attr = prim.GetAttribute(name)
    if attr and attr.IsValid():
        attr.Set(value)
    else:
        print("  [WARN] attribute not found: %s on %s" % (name, prim.GetPath()))


def _attr_get(prim, name):
    attr = prim.GetAttribute(name)
    return attr.Get() if (attr and attr.IsValid()) else None


# ==============================================================================
# Fix functions
# ==============================================================================

def _fix_lock_joints(stage):
    """
    For each joint in LOCK_JOINTS:
      - Set high stiffness + damping so the joint holds its current position.
      - Set targetPosition = current state position (hold-in-place).
    """
    print("\n[FixPhysics] Locking non-target joints...")
    fixed_count = 0

    for path, stiff, damp, mf in LOCK_JOINTS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print("  [WARN] not found: " + path)
            continue

        name = path.split("/")[-1]

        # Determine drive namespace (angular for revolute, linear for prismatic)
        if prim.IsA(UsdPhysics.RevoluteJoint):
            ns = "drive:angular:physics"
            state_ns = "state:angular:physics:position"
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            ns = "drive:linear:physics"
            state_ns = "state:linear:physics:position"
        else:
            print("  [WARN] unknown joint type: " + name)
            continue

        # Read current position to use as target (hold in place)
        cur = _attr_get(prim, state_ns)
        target = cur if cur is not None else 0.0

        _attr_set(prim, ns + ":stiffness",      stiff)
        _attr_set(prim, ns + ":damping",        damp)
        _attr_set(prim, ns + ":maxForce",       mf)
        _attr_set(prim, ns + ":targetPosition", target)
        _attr_set(prim, ns + ":type",           "force")

        print("  LOCK  %-45s  stiff=%.0f  damp=%.0f  target=%.4f" % (
            name, stiff, damp, target))
        fixed_count += 1

    return fixed_count


def _fix_slide_joint(stage):
    """Fix the prismatic joint limits (original USD has [0, 0])."""
    print("\n[FixPhysics] Fixing prismatic joint limits...")
    prim = stage.GetPrimAtPath(SLIDE_JOINT_PATH)
    if not prim.IsValid():
        print("  [ERROR] Slide joint not found: " + SLIDE_JOINT_PATH)
        return False

    lo_before = _attr_get(prim, "physics:lowerLimit")
    hi_before = _attr_get(prim, "physics:upperLimit")

    _attr_set(prim, "physics:lowerLimit", SLIDE_STROKE_MIN)
    _attr_set(prim, "physics:upperLimit", SLIDE_STROKE_MAX)

    print("  lowerLimit:  %.4f -> %.4f m" % (
        lo_before if lo_before is not None else 0.0, SLIDE_STROKE_MIN))
    print("  upperLimit:  %.4f -> %.4f m" % (
        hi_before if hi_before is not None else 0.0, SLIDE_STROKE_MAX))
    return True


def _check_fixed_base(stage):
    """
    fixed_base has mass=0 which is fine (it's a world anchor),
    but report it so the user knows it's intentional.
    """
    prim = stage.GetPrimAtPath("/World/Crusher_IsaacSim/fixed_base")
    if prim.IsValid() and prim.HasAPI(UsdPhysics.MassAPI):
        mass = _attr_get(prim, "physics:mass")
        if mass is not None and mass == 0.0:
            print("\n[FixPhysics] NOTE: fixed_base has mass=0 (world anchor - OK).")


# ==============================================================================
# Main entry point
# ==============================================================================

def fix_physics():
    """
    Apply all physics fixes. Call this before Play or before Drive_SlideJoint.
    Safe to call multiple times.
    """
    stage = _stage()
    if stage is None:
        print("[ERROR] No stage loaded. Open Crushing.usd first.")
        return

    print("=" * 56)
    print("  FixPhysics - stabilizing articulation")
    print("=" * 56)

    n = _fix_lock_joints(stage)
    ok = _fix_slide_joint(stage)
    _check_fixed_base(stage)

    print("\n[FixPhysics] Done.")
    print("  Locked %d joints.  Slide joint limits: [%.0f, %.0f] mm" % (
        n, SLIDE_STROKE_MIN * 1000, SLIDE_STROKE_MAX * 1000))
    print("=" * 56)
    if ok:
        print("\n  Next steps:")
        print("    1. Press Play (if not already playing)")
        print("    2. Run Drive_SlideJoint.py -> start()")


def status():
    """Print current drive params for all joints being managed."""
    stage = _stage()
    if stage is None:
        print("[ERROR] No stage.")
        return

    all_paths = [p for p, *_ in LOCK_JOINTS] + [SLIDE_JOINT_PATH]
    print("=" * 72)
    print("  %-45s  %-10s  %-8s  %-8s" % ("joint", "type", "stiff", "damp"))
    print("-" * 72)
    for path in all_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print("  NOT FOUND: " + path)
            continue
        name = path.split("/")[-1]
        if prim.IsA(UsdPhysics.RevoluteJoint):
            ns = "drive:angular:physics"
        else:
            ns = "drive:linear:physics"
        stiff = _attr_get(prim, ns + ":stiffness")
        damp  = _attr_get(prim, ns + ":damping")
        tgt   = _attr_get(prim, ns + ":targetPosition")
        jtype = "Revolute" if prim.IsA(UsdPhysics.RevoluteJoint) else "Prismatic"
        print("  %-45s  %-10s  %-8s  %-8s  target=%.4f" % (
            name, jtype,
            ("%.1f" % stiff) if stiff is not None else "None",
            ("%.4f" % damp)  if damp  is not None else "None",
            tgt if tgt is not None else 0.0))
    print("=" * 72)


# ==============================================================================
# Auto-run
# ==============================================================================

print()
print("=" * 56)
print("  FixPhysics.py  loaded")
print("=" * 56)
print("  fix_physics()   apply all stability fixes")
print("  status()        show current joint params")
print("=" * 56)
print()

fix_physics()

"""
MakeCollisionFree.py
====================
Run with the isaacsim conda environment (offline, no Isaac Sim needed):

  C:\\Users\\simuser\\anaconda3\\envs\\isaacsim\\python.exe MakeCollisionFree.py

Reads  : Crushing.usd
Writes : Crushing_CollisionFree.usd  (disk patched, original untouched)

Strategy
--------
Internal mechanism parts (gears, bearings, shafts, keys) that are
geometrically interpenetrating at simulation start have their collision
DISABLED.  External structural parts (base, walls, guides, plate) keep
collision ENABLED so the robot still interacts with the environment.

After this step the simulation will be stable.
Next step: run CoACD convex decomposition on the gear meshes and
re-enable collision with proper convex-hull approximations.
"""

import os
from pxr import Usd, UsdPhysics, Sdf

# ── Paths ─────────────────────────────────────────────────────────────────────
_DIR      = os.path.dirname(os.path.abspath(__file__))
IN_USD    = os.path.join(_DIR, "..", "Crusher_IsaacSim_description", "Crushing.usd")
OUT_USD   = os.path.join(_DIR, "..", "Crusher_IsaacSim_description", "Crushing_CollisionFree.usd")

# ── Links whose collision is DISABLED (internal mechanism parts) ──────────────
#
#  Why these?
#  - Gears (Rack, Spur, Bevel*): physically mesh/overlap with each other.
#  - Bearings / Snaprings / Keys: assembled inside housings with near-zero
#    clearance; any mesh inaccuracy causes interpenetration.
#  - Shafts / Couplers inside housings: same reason.
#  - Motor bodies: wrapped by brackets, overlap in assembly.
#
#  Structural parts (base_link, walls, guides, plate, case) are kept as-is.
# ──────────────────────────────────────────────────────────────────────────────
COLLISION_FREE_LINKS = {
    # ── Gear pairs (directly interpenetrating) ────────────────────────────────
    "L3_RackGear_1",            # rack  ↔ spur gear
    "L7_SpurGear_1",            # spur  ↔ rack gear

    # ── Bevel gear mechanism ──────────────────────────────────────────────────
    "L3_Bevel_GearBox_1",       # bevel gear housing (meshes with L4_Bevel_GearBox2_1)
    "L4_Bevel_GearBox2_1",      # bevel output gear

    # ── Motor 1 drivetrain (shaft + bearings + snaprings + key) ───────────────
    "L4_Shaft_1",               # main shaft (inside bevel gearbox)
    "L4_Motor1_FrontBearing_1", # bearing in housing
    "L4_Motor1_RearBearing_1",
    "L5_Motor1_FrontSnapring_1",# snapring on bearing
    "L5_Motor1_RearSnapring_1",
    "L5_Key_1",                 # shaft key (inside keyway slot)

    # ── Reducer / Motor 1 body ────────────────────────────────────────────────
    "L4_Reducer1_1",            # reducer body (assembled tightly)
    "L5_Reducer2_1",
    "L6_Motor1_Bolt_1",         # mounting bolt (inside bracket)
    "L7_Motor1_Body_1",         # motor body (inside bracket)

    # ── Motor 2 drivetrain ────────────────────────────────────────────────────
    "L4_Motor2_Shaft_1",        # motor 2 shaft (inside coupler)
    "L5_Coupler_1",             # coupler (wraps shaft)
    "L6_DcutShaft_1",           # D-cut shaft (inside bearings)
    "L7_Holder_Bearing1_1",     # bearing (on shaft)
    "L7_Holder_Bearing2_1",

    # ── Arm linkage ───────────────────────────────────────────────────────────
    # These links pass through or very close to each other during motion.
    # Joint constraints handle the kinematics; collision only causes instability.
    "L5_Link1_1",
    "L6_Link2_1",
    "L7_Link3_1",
    "L8_Link3_Shaft_1",

    # ── Linear slide internals ────────────────────────────────────────────────
    "L2_Linear_bush_1",         # linear bush (slides inside guide)
}

# ── Links that KEEP collision (external / structural) ─────────────────────────
# base_link, L1_*, L2_Wall*, L2_GearCase*, L2__Motor2_Braket_1,
# L3_GearCase1_1, L3_Motor2_1, L6_Motor1_Braket_1, L9_PLATE_v3_1,
# L1_Guide1_1, L2_Left_Wall1_1, L1_Slider_jig_1, L1_Wall*


def _collision_prim_path(link_name: str) -> str:
    """Build the full path to the collision mesh prim for a given link."""
    return (
        f"/World/Crusher_IsaacSim/{link_name}"
        f"/collisions/{link_name}/node_STL_BINARY_"
    )


def _fix_nested_articulation_root(stage) -> int:
    """
    Remove ArticulationRootAPI from any prim whose ancestor already has it.
    PhysX allows only ONE ArticulationRoot per kinematic chain.
    Returns the number of prims fixed.
    """
    # Collect all ArticulationRoot paths first
    root_paths = []
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            root_paths.append(prim.GetPath())

    fixed = []
    for path in root_paths:
        prim = stage.GetPrimAtPath(path)
        # Walk up the hierarchy to check if any ancestor is also a root
        ancestor = prim.GetParent()
        while ancestor and ancestor.GetPath() != Sdf.Path.absoluteRootPath:
            if ancestor.HasAPI(UsdPhysics.ArticulationRootAPI):
                # This prim is nested under another root -> remove its API
                UsdPhysics.ArticulationRootAPI(prim).Apply(prim)  # ensure applied first
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                fixed.append(str(path))
                break
            ancestor = ancestor.GetParent()

    return fixed


def make_collision_free():
    print("=" * 60)
    print("  MakeCollisionFree.py")
    print("=" * 60)
    print(f"  Input  : {IN_USD}")
    print(f"  Output : {OUT_USD}")
    print()

    # Open source stage
    stage = Usd.Stage.Open(IN_USD)
    if not stage:
        print("[ERROR] Cannot open: " + IN_USD)
        return

    # Fix nested ArticulationRoot
    fixed_roots = _fix_nested_articulation_root(stage)
    if fixed_roots:
        print(f"  Removed nested ArticulationRootAPI from:")
        for p in fixed_roots:
            print(f"    - {p}")
    else:
        print("  No nested ArticulationRoot found.")
    print()

    disabled = []
    not_found = []

    for link_name in sorted(COLLISION_FREE_LINKS):
        coll_path = _collision_prim_path(link_name)
        prim = stage.GetPrimAtPath(coll_path)

        if not prim.IsValid():
            not_found.append(link_name)
            continue

        # Set collisionEnabled = False
        attr = prim.GetAttribute("physics:collisionEnabled")
        if attr and attr.IsValid():
            attr.Set(False)
            disabled.append(link_name)
        else:
            not_found.append(link_name + " (no attr)")

    # ── Report ────────────────────────────────────────────────────────────────
    print(f"  Disabled collision on {len(disabled)} links:")
    for name in disabled:
        print(f"    - {name}")

    if not_found:
        print(f"\n  [WARN] Not found ({len(not_found)}):")
        for name in not_found:
            print(f"    - {name}")

    # ── Save ─────────────────────────────────────────────────────────────────
    stage.Export(OUT_USD)
    print(f"\n  Saved -> {OUT_USD}")
    print()
    print("  Next steps:")
    print("    1. Open Crushing_CollisionFree.usd in Isaac Sim")
    print("    2. Run FixPhysics.py  (Stop -> Fix -> Play)")
    print("    3. Run Drive_SlideJoint.py -> start()")
    print()
    print("  For Step 2 (CoACD):")
    print("    Run the gear STL files through CoACD to get convex-decomposed")
    print("    collision meshes, then re-enable collision on gear links only.")
    print("=" * 60)


if __name__ == "__main__":
    make_collision_free()

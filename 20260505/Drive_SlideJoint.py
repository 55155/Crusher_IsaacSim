"""
Drive_SlideJoint.py  -  Isaac Sim 4.5 / 5.x  Script Editor
============================================================
Target joint : L1_Guide1_1_L2_Left_Wall1_1  (Prismatic / linear slider)
Payload      : ~200 g  ->  actuation force kept small

How to run
----------
1. Open Isaac Sim
2. File > Open  ->  C:/TEMP/Crusher_IsaacSim_description/Crushing.usd
3. Press Play (triangle button)
4. Window > Script Editor
5. Paste this entire script and click [Run]
   -> The slider starts reciprocating automatically.

Commands available after loading (run each line separately):
  stop()          Stop motion and return to home (0 mm)
  start()         Restart reciprocating motion
  move_to(15)     Move to 15 mm (single shot, mm unit)
  status()        Print current joint state
"""

import asyncio
import math
import time

import omni.usd
import omni.kit.app
import omni.timeline
from pxr import UsdPhysics

# ==============================================================================
# Parameters  (edit here)
# ==============================================================================

JOINT_PATH = "/World/Crusher_IsaacSim/joints/L1_Guide1_1_L2_Left_Wall1_1"

STROKE_MIN_M = 0.000   # home position [m]  (0 mm)
STROKE_MAX_M = 0.010   # max stroke    [m]  (30 mm)
CYCLE_PERIOD = 4.0     # reciprocating period [s]

# Drive parameters  (SI units, stage is 1 m = 1 unit)
#   F_restore = stiffness * position_error
#   At 30 mm error -> 500 * 0.03 = 15 N  (clamped by max_force)
DRIVE_STIFFNESS = 500.0   # [N/m]
DRIVE_DAMPING   = 15.0    # [N*s/m]  critical damping ~= 2*sqrt(500*0.2) ~= 20
DRIVE_MAX_FORCE =  5.0    # [N]      ~2.5x gravity load of 200 g

# ==============================================================================
# Internal state
# ==============================================================================

_task    = None
_running = False
_t0      = None   # wall-clock start time


# ==============================================================================
# USD helpers
# ==============================================================================

def _stage():
    return omni.usd.get_context().get_stage()


def _joint():
    stage = _stage()
    if stage is None:
        return None
    prim = stage.GetPrimAtPath(JOINT_PATH)
    return prim if prim.IsValid() else None


def _attr_set(prim, name, value):
    attr = prim.GetAttribute(name)
    if attr and attr.IsValid():
        attr.Set(value)


def _attr_get(prim, name):
    attr = prim.GetAttribute(name)
    return attr.Get() if (attr and attr.IsValid()) else None


def _setup_drive() -> bool:
    prim = _joint()
    if prim is None:
        print("[ERROR] Joint not found: " + JOINT_PATH)
        print("        Make sure the USD is loaded and the path is correct.")
        return False

    if not prim.IsA(UsdPhysics.PrismaticJoint):
        print("[ERROR] Not a PrismaticJoint: " + JOINT_PATH)
        return False

    # stroke limits  [m]
    _attr_set(prim, "physics:lowerLimit", STROKE_MIN_M)
    _attr_set(prim, "physics:upperLimit", STROKE_MAX_M)

    # linear drive API  [SI]
    _attr_set(prim, "drive:linear:physics:stiffness", DRIVE_STIFFNESS)
    _attr_set(prim, "drive:linear:physics:damping",   DRIVE_DAMPING)
    _attr_set(prim, "drive:linear:physics:maxForce",  DRIVE_MAX_FORCE)
    _attr_set(prim, "drive:linear:physics:type",      "force")

    print("  stiffness = %.1f N/m" % DRIVE_STIFFNESS)
    print("  damping   = %.1f N*s/m" % DRIVE_DAMPING)
    print("  max_force = %.1f N" % DRIVE_MAX_FORCE)
    print("  stroke    = [%.0f, %.0f] mm" % (STROKE_MIN_M * 1000, STROKE_MAX_M * 1000))
    return True


def _set_target(pos_m: float):
    prim = _joint()
    if prim is None:
        return
    clamped = max(STROKE_MIN_M, min(STROKE_MAX_M, pos_m))
    _attr_set(prim, "drive:linear:physics:targetPosition", clamped)


# ==============================================================================
# Async motion loop  (Isaac Sim Script Editor standard pattern)
# ==============================================================================

async def _motion_loop():
    global _running, _t0

    app      = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()

    _t0      = time.time()
    _running = True
    _log_sec = -1

    while _running:
        await app.next_update_async()

        if not timeline.is_playing():
            _t0 += 1.0 / 60.0   # freeze timer while paused
            continue

        elapsed = time.time() - _t0

        # smooth cosine ramp: 0 -> max -> 0 -> ...
        phase  = (elapsed / CYCLE_PERIOD) * 2.0 * math.pi
        ratio  = (1.0 - math.cos(phase)) / 2.0
        target = STROKE_MIN_M + ratio * (STROKE_MAX_M - STROKE_MIN_M)

        _set_target(target)

        # log once per second
        sec = int(elapsed)
        if sec != _log_sec:
            _log_sec = sec
            prim = _joint()
            cur = _attr_get(prim, "state:linear:physics:position") if prim else 0.0
            cur = cur if cur is not None else 0.0
            print("  [t=%5.1fs]  target=%5.1f mm   current=%5.1f mm" % (
                elapsed, target * 1000, cur * 1000))


# ==============================================================================
# Public API
# ==============================================================================

def start():
    global _task, _running

    if _task is not None and not _task.done():
        _running = False
        _task.cancel()

    print("\n[SlideJoint] Setting up drive parameters...")
    if not _setup_drive():
        return

    print("[SlideJoint] Starting reciprocating motion. Call stop() to halt.\n")
    _task = asyncio.ensure_future(_motion_loop())


def stop():
    global _running, _task

    _running = False
    if _task is not None and not _task.done():
        _task.cancel()
    _task = None

    _set_target(STROKE_MIN_M)
    print("[SlideJoint] Stopped. Returning to home (0 mm).")


def move_to(position_mm: float):
    if not _setup_drive():
        return
    _set_target(position_mm / 1000.0)
    print("[SlideJoint] Target -> %.1f mm" % position_mm)


def status():
    prim = _joint()
    if prim is None:
        print("[SlideJoint] Joint not found.")
        return

    cur  = _attr_get(prim, "state:linear:physics:position")
    vel  = _attr_get(prim, "state:linear:physics:velocity")
    tgt  = _attr_get(prim, "drive:linear:physics:targetPosition")
    stif = _attr_get(prim, "drive:linear:physics:stiffness")
    damp = _attr_get(prim, "drive:linear:physics:damping")
    mf   = _attr_get(prim, "drive:linear:physics:maxForce")
    lo   = _attr_get(prim, "physics:lowerLimit")
    hi   = _attr_get(prim, "physics:upperLimit")

    mm = lambda v: ("%.2f mm" % (float(v) * 1000)) if v is not None else "N/A"

    print("=" * 48)
    print("  Joint   : " + JOINT_PATH.split("/")[-1])
    print("  Running : " + ("YES" if _running else "NO"))
    print("-" * 48)
    print("  position   : " + mm(cur))
    print("  velocity   : " + mm(vel) + "/s")
    print("  target     : " + mm(tgt))
    print("  stiffness  : " + (("%.2f N/m" % stif) if stif is not None else "N/A"))
    print("  damping    : " + (("%.4f N*s/m" % damp) if damp is not None else "N/A"))
    print("  max_force  : " + (("%.2f N" % mf) if mf is not None else "N/A"))
    print("  stroke     : [" + mm(lo) + " ~ " + mm(hi) + "]")
    print("=" * 48)


# ==============================================================================
# Auto-execute when [Run] is pressed in Script Editor
# ==============================================================================

print()
print("=" * 48)
print("  Drive_SlideJoint.py  loaded")
print("=" * 48)
print("  start()       start reciprocating motion")
print("  stop()        stop + return to home")
print("  move_to(mm)   move to position in mm")
print("  status()      print joint state")
print("=" * 48)
print()

_tl = omni.timeline.get_timeline_interface()
if _tl.is_playing():
    start()
else:
    print("  NOTE: Simulation is not playing.")
    print("  Press Play (triangle), then call start().")
    print()

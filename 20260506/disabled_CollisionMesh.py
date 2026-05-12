"""
disabled_CollisionMesh.py  -  Isaac Sim Script Editor
------------------------------------------------------
Disables collision on ALL meshes in the current stage.
"""

import omni.usd
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()

count = 0
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        attr = prim.GetAttribute("physics:collisionEnabled")
        if attr and attr.IsValid():
            attr.Set(False)
            count += 1

print("Collision disabled on %d prims." % count)

'''
## Collision-Free Checking
from pxr import Usd, UsdPhysics

stage = Usd.Stage.Open(r"C:\TEMP\Crusher_IsaacSim_description\Crushing_CollisionFree.usd")

for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        attr = prim.GetAttribute("physics:collisionEnabled")
        enabled = attr.Get() if attr and attr.IsValid() else "not set"
        status = "ON " if enabled else "OFF"
        link = str(prim.GetPath()).split("/")[3]   # link name
        print(f"  [{status}]  {link}")
'''
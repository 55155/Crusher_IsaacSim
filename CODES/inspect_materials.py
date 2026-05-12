from pxr import Usd, UsdShade, UsdGeom

path = 'C:/TEMP/Crusher_IsaacSim_description/urdf/Crusher_IsaacSim/Crusher_IsaacSim.usd'
stage = Usd.Stage.Open(path)

print("=== Materials / Shaders ===")
for prim in stage.TraverseAll():
    if prim.GetTypeName() in ('Material', 'Shader'):
        print(f"  [{prim.GetTypeName()}] {prim.GetPath()}")
        if prim.GetTypeName() == 'Shader':
            for attr in prim.GetAttributes():
                val = attr.Get()
                if val is not None:
                    print(f"      {attr.GetName()} = {val}")

print()
print("=== MaterialBinding on Meshes (first 5) ===")
count = 0
for prim in stage.TraverseAll():
    if prim.GetTypeName() == 'Mesh':
        binding = UsdShade.MaterialBindingAPI(prim).GetDirectBinding()
        mat_path = binding.GetMaterialPath()
        print(f"  {prim.GetName()} → {mat_path if mat_path else '(없음)'}")
        count += 1
        if count >= 5:
            break

print()
print("=== configuration 파일 목록 ===")
import os
conf_dir = 'C:/TEMP/Crusher_IsaacSim_description/urdf/Crusher_IsaacSim/configuration'
for f in sorted(os.listdir(conf_dir)):
    size = os.path.getsize(os.path.join(conf_dir, f))
    print(f"  {f}  ({size:,} bytes)")

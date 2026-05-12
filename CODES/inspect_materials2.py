from pxr import Usd, UsdShade, UsdGeom

# base.usd 직접 열어서 메쉬-머티리얼 바인딩 확인
base_path = 'C:/TEMP/Crusher_IsaacSim_description/urdf/Crusher_IsaacSim/configuration/Crusher_IsaacSim_base.usd'
stage = Usd.Stage.Open(base_path)

print("=== Materials in base.usd ===")
for prim in stage.TraverseAll():
    if prim.GetTypeName() == 'Material':
        print(f"  {prim.GetPath()}")

print()
print("=== Shaders in base.usd ===")
for prim in stage.TraverseAll():
    if prim.GetTypeName() == 'Shader':
        print(f"  {prim.GetPath()}")
        for attr in prim.GetAttributes():
            v = attr.Get()
            if v is not None:
                print(f"      {attr.GetName()} = {v}")

print()
print("=== MaterialBinding on Meshes (all) ===")
count = 0
for prim in stage.TraverseAll():
    if prim.GetTypeName() == 'Mesh':
        binding = UsdShade.MaterialBindingAPI(prim).GetDirectBinding()
        mat = binding.GetMaterialPath()
        print(f"  {prim.GetPath().name} → {mat if mat else '(바인딩 없음)'}")
        count += 1
        if count >= 5:
            print(f"  ... (총 mesh 수 확인 중)")
            break

# 전체 Mesh 수
all_meshes = [p for p in stage.TraverseAll() if p.GetTypeName() == 'Mesh']
print(f"\n  총 Mesh 수: {len(all_meshes)}")

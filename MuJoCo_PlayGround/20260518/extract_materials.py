"""
extract_materials.py
====================
Crushing_CollisionFree.usd 에서 링크별 Material 정보 추출.
mesh 레벨 바인딩 기준으로 수집.

실행:
  conda activate isaacsim
  python extract_materials.py
"""

import json
from pathlib import Path
from pxr import Usd, UsdShade, UsdGeom

USD_PATH = r"C:\Crusher_isaacsim\Crusher_IsaacSim_description\Crushing_CollisionFree.usd"
OUT_PATH = r"C:\Crusher_isaacsim\MuJoCo_PlayGround\20260518\materials.json"

stage = Usd.Stage.Open(USD_PATH)

# ── 1. 고유 Material → Shader 속성 수집 ──────────────────────────────────────
def collect_materials():
    mats = {}
    for prim in stage.Traverse():
        if prim.GetTypeName() != "Material":
            continue
        name = prim.GetName()
        if name in mats:
            continue
        info = {}
        for child in prim.GetAllChildren():
            if child.GetTypeName() != "Shader":
                continue
            for attr in child.GetAttributes():
                val = attr.Get()
                if val is None:
                    continue
                n = attr.GetName()
                if any(k in n.lower() for k in [
                    "diffuse","albedo","color","metallic",
                    "roughness","emissive","opacity","tint",
                    "brightness","ior","specular"
                ]):
                    if hasattr(val, "__iter__") and not isinstance(val, str):
                        val = list(val)
                    info[n] = val
        mats[name] = info
    return mats

# ── 2. 링크별 mesh Material 바인딩 수집 ──────────────────────────────────────
def collect_link_bindings(materials):
    """
    USD 구조:
      /World/Crusher_IsaacSim/<LinkName>/visuals/node_.../Looks/<MaterialName>/mesh
    링크 이름(부모 체인에서 추출) + mesh 에 바인딩된 Material 수집.
    """
    bindings = {}

    for prim in stage.Traverse():
        if prim.GetName() != "mesh":
            continue

        api = UsdShade.MaterialBindingAPI(prim)
        mat, _ = api.ComputeBoundMaterial()
        if not mat:
            continue

        mat_name = mat.GetPrim().GetName()
        if mat_name == "BlackMaterial":   # collision mesh 제외
            continue

        # 링크 이름 추출: path = /.../Crusher_IsaacSim/<LinkName>/...
        parts = str(prim.GetPath()).split("/")
        try:
            robot_idx = parts.index("Crusher_IsaacSim")
            link_name = parts[robot_idx + 1]
        except (ValueError, IndexError):
            continue

        mat_info = materials.get(mat_name, {})
        diffuse = next(
            (v for k, v in mat_info.items()
             if "diffuse" in k.lower() or "albedo" in k.lower()),
            [0.5, 0.5, 0.5]
        )

        bindings[link_name] = {
            "material": mat_name,
            "diffuse":  diffuse,
        }

    return bindings


materials = collect_materials()
bindings  = collect_link_bindings(materials)

# ── 3. 출력 ──────────────────────────────────────────────────────────────────
print(f"고유 Material: {len(materials)}개")
print(f"링크 바인딩:   {len(bindings)}개\n")

print(f"{'링크':<35} {'Material':<25} diffuse")
print("-" * 80)
for link, info in sorted(bindings.items()):
    print(f"  {link:<33} {info['material']:<25} {info['diffuse']}")

# ── 4. JSON 저장 ──────────────────────────────────────────────────────────────
output = {"materials": materials, "bindings": bindings}
Path(OUT_PATH).parent.mkdir(parents=True, exist_ok=True)
with open(OUT_PATH, "w", encoding="utf-8") as f:
    json.dump(output, f, indent=2, default=str)

print(f"\n→ 저장: {OUT_PATH}")

"""
apply_to_mjcf.py
================
appearances.json (Fusion 360) -> Crusher_IsaacSim_colored.xml

실행:
  conda activate isaacsim
  python apply_to_mjcf.py
"""

import json
import re
import xml.etree.ElementTree as ET
import xml.dom.minidom
from pathlib import Path

FUSION_JSON = r"C:\Crusher_isaacsim\MuJoCo_PlayGround\20260518\fusion360\appearances.json"
MJCF_IN     = r"C:\Crusher_isaacsim\MuJoCo_PlayGround\MJCF\Crusher_IsaacSim.xml"
MJCF_OUT    = r"C:\Crusher_isaacsim\MuJoCo_PlayGround\MJCF\Crusher_IsaacSim_colored.xml"


# ── 1. Appearance 이름/속성에서 RGBA 추출 ─────────────────────────────────────

def parse_rgba(body_info):
    """
    우선순위:
      1) appearance_name 에서 Opaque(R,G,B) 파싱  ← 가장 신뢰할 수 있는 값
      2) appearance_name 에서 PSOLRGB-R-G-B 파싱
      3) 이름 기반 근사값 (재질명 fallback)
      4) ColorProperty — Fusion 360 에서 항상 [255,255,255,255](무색조)로 오므로 최후 수단

    주의: Fusion 360의 '색상' ColorProperty 는 색조(tint) 배율이며
          흰색(255,255,255)이면 "재질 기본색 사용"을 의미하므로 신뢰도가 낮다.
    """
    name = body_info.get("appearance_name", "")

    # 1) "Opaque(R,G,B)" 패턴
    m = re.search(r"Opaque\s*\((\d+),\s*(\d+),\s*(\d+)\)", name)
    if m:
        r, g, b = int(m.group(1)), int(m.group(2)), int(m.group(3))
        return [round(r/255, 4), round(g/255, 4), round(b/255, 4), 1.0]

    # 2) "PSOLRGB-R-G-B" 패턴
    m = re.search(r"PSOLRGB[- ](\d+)[- ](\d+)[- ](\d+)", name)
    if m:
        r, g, b = int(m.group(1)), int(m.group(2)), int(m.group(3))
        return [round(r/255, 4), round(g/255, 4), round(b/255, 4), 1.0]

    # 3) 이름 기반 근사 (재질명 fallback)
    NAME_FALLBACK = {
        "강철 - 새틴":    [0.55, 0.55, 0.58, 1.0],
        "알루미늄 - 연마": [0.82, 0.82, 0.82, 1.0],
        "철 - 주조":      [0.35, 0.33, 0.32, 1.0],
        "ABS(흰색)":      [0.95, 0.95, 0.95, 1.0],
        "EPX 150 - 에어 베이크드(탄소 M2 M3 L1 3D 프린터 포함)": [0.10, 0.10, 0.10, 1.0],
    }
    for key, rgba in NAME_FALLBACK.items():
        if key in name:
            return rgba

    # 4) ColorProperty — tint 가 실제 색인 경우에만 유효 (흰색이면 무시)
    for key, val in body_info.items():
        if isinstance(val, dict) and val.get("type") == "color":
            r, g, b, a = val["rgba"]
            if not (r == 255 and g == 255 and b == 255):  # 흰색(무색조)은 건너뜀
                return [round(r/255, 4), round(g/255, 4), round(b/255, 4), round(a/255, 4)]

    return [0.5, 0.5, 0.5, 1.0]  # 기본 회색


def get_material_props(body_info):
    """roughness, metallic → MuJoCo specular/shininess/reflectance."""
    roughness = 0.5
    metallic  = 0.0

    for key, val in body_info.items():
        if not isinstance(val, dict):
            continue
        if val.get("type") == "float":
            k = key.lower()
            if "rough" in k:
                roughness = val["value"]
            elif "metal" in k:
                metallic = val["value"]

    specular    = round(metallic * 0.9 + (1 - metallic) * 0.1, 3)
    shininess   = round((1 - roughness) ** 2, 3)
    reflectance = round(metallic * 0.6, 3)
    return specular, shininess, reflectance


# ── 2. Fusion 컴포넌트명 → MJCF 링크명 매핑 ──────────────────────────────────
# Fusion: "7_SpurGear"  →  MJCF: "L7_SpurGear_1"
# Fusion: "base_link"   →  MJCF: "base_link"

def fusion_to_mjcf_name(fusion_name):
    """
    "7_SpurGear"      → "L7_SpurGear_1"
    "2_ Motor2_Braket" → "L2__Motor2_Braket_1"  (공백 제거)
    "base_link"        → "base_link"
    """
    name = fusion_name.strip()
    if name == "base_link":
        return "base_link"
    # 숫자로 시작하면 L 붙이고 _1 추가
    m = re.match(r"^(\d+)_(.+)$", name.replace(" ", ""))
    if m:
        return f"L{m.group(1)}_{m.group(2)}_1"
    return name


# ── 3. JSON 로드 및 링크별 material 생성 ─────────────────────────────────────

with open(FUSION_JSON, encoding="utf-8") as f:
    fusion_data = json.load(f)

link_materials = {}  # mjcf_link_name → material attrs

for comp_name, bodies in fusion_data.items():
    mjcf_name = fusion_to_mjcf_name(comp_name)
    # 첫 번째 body 기준
    for body_name, body_info in bodies.items():
        rgba                    = parse_rgba(body_info)
        specular, shininess, reflectance = get_material_props(body_info)
        link_materials[mjcf_name] = {
            "rgba":        f"{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}",
            "specular":    str(specular),
            "shininess":   str(shininess),
            "reflectance": str(reflectance),
            "emission":    "0.0",
        }
        break

print(f"링크-Material 매핑: {len(link_materials)}개")
for k, v in sorted(link_materials.items()):
    print(f"  {k:<35} rgba={v['rgba']}")


# ── 4. MJCF 수정 ──────────────────────────────────────────────────────────────

tree = ET.parse(MJCF_IN)
root = tree.getroot()

asset = root.find("asset")
if asset is None:
    asset = ET.SubElement(root, "asset")

for m in asset.findall("material"):
    asset.remove(m)

for link_name, attrs in link_materials.items():
    ET.SubElement(asset, "material", name=link_name, **attrs)

# geom 에 material 연결 (mesh 이름 기준)
for geom in root.iter("geom"):
    if "material" in geom.attrib:
        del geom.attrib["material"]
    mesh = geom.get("mesh", "")
    if mesh in link_materials:
        geom.set("material", mesh)

# ── 5. 저장 ───────────────────────────────────────────────────────────────────

xml_str = ET.tostring(root, encoding="unicode")
pretty  = xml.dom.minidom.parseString(xml_str).toprettyxml(indent="  ")
lines   = [l for l in pretty.splitlines() if l.strip()]

with open(MJCF_OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines))

print(f"\n저장: {MJCF_OUT}")

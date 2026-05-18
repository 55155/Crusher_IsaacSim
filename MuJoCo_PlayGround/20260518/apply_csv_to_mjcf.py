"""
apply_csv_to_mjcf.py
====================
mesh_colors.csv (r,g,b 채워진 것) → MJCF material 적용.
Fusion 360 JSON 없이도 사용 가능.

실행:
  conda activate isaacsim
  python apply_csv_to_mjcf.py
"""

import csv
import xml.etree.ElementTree as ET
import xml.dom.minidom
from pathlib import Path

CSV_PATH = r"C:\Crusher_isaacsim\MuJoCo_PlayGround\20260518\mesh_colors.csv"
MJCF_IN  = r"C:\Crusher_isaacsim\MuJoCo_PlayGround\MJCF\Crusher_IsaacSim.xml"
MJCF_OUT = r"C:\Crusher_isaacsim\MuJoCo_PlayGround\MJCF\Crusher_IsaacSim_colored.xml"

# ── CSV 로드 ──────────────────────────────────────────────────────────────────
materials = {}
with open(CSV_PATH, encoding="utf-8-sig") as f:
    for row in csv.DictReader(f):
        # CSV는 0-255 정수, MuJoCo는 0-1 소수점 필요 → 255로 나눔
        r = round(float(row["r"].strip() or 128) / 255, 4)
        g = round(float(row["g"].strip() or 128) / 255, 4)
        b = round(float(row["b"].strip() or 128) / 255, 4)
        a = round(float(row["a"].strip() or 1.0), 4)
        materials[row["link"]] = {
            "rgba":        f"{r} {g} {b} {a}",
            "specular":    row["specular"],
            "shininess":   row["shininess"],
            "reflectance": row["reflectance"],
            "emission":    row["emission"],
        }

print(f"CSV material: {len(materials)}개")

# ── MJCF 수정 ─────────────────────────────────────────────────────────────────
tree = ET.parse(MJCF_IN)
root = tree.getroot()

asset = root.find("asset")
if asset is None:
    asset = ET.SubElement(root, "asset")

for m in asset.findall("material"):
    asset.remove(m)

# material 정의 추가
for link_name, attrs in materials.items():
    ET.SubElement(asset, "material", name=link_name, **attrs)

# geom에 material 연결
for geom in root.iter("geom"):
    if "material" in geom.attrib:
        del geom.attrib["material"]

    mesh = geom.get("mesh", "")
    if mesh in materials:
        geom.set("material", mesh)

# ── 조명 추가 ─────────────────────────────────────────────────────────────────
worldbody = root.find("worldbody")
for light in worldbody.findall("light"):
    worldbody.remove(light)

LIGHTS = [
    {"name": "ambient_top", "pos": "0 0 2",      "dir": "0 0 -1",       "diffuse": "0.8 0.8 0.8", "specular": "0.3 0.3 0.3", "directional": "true", "castshadow": "false"},
    {"name": "front",       "pos": "0.5 -1.5 1.5","dir": "-0.2 0.7 -0.6","diffuse": "0.6 0.6 0.6", "specular": "0.2 0.2 0.2", "directional": "true", "castshadow": "false"},
    {"name": "back",        "pos": "-0.5 1.5 1.0","dir": "0.2 -0.6 -0.5","diffuse": "0.4 0.4 0.4", "specular": "0.1 0.1 0.1", "directional": "true", "castshadow": "false"},
]
for i, attrs in enumerate(LIGHTS):
    worldbody.insert(i, ET.Element("light", **attrs))

# ── 저장 ──────────────────────────────────────────────────────────────────────
xml_str = ET.tostring(root, encoding="unicode")
pretty  = xml.dom.minidom.parseString(xml_str).toprettyxml(indent="  ")
lines   = [l for l in pretty.splitlines() if l.strip()]
with open(MJCF_OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines))

print(f"저장: {MJCF_OUT}")

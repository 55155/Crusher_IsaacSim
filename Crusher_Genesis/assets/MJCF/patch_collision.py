"""
patch_collision.py
Crusher_IsaacSim_colored.xml 의 geom 충돌 설정을 일괄 수정합니다.

규칙:
  충돌 유지 (원본 유지) :
    L1_Wall1_1, L1_Wall2_1, L2_Wall3_1, L9_PLATE_v3_1,
    L2_Left_Wall1_1 CoACD hull 들, ground

  나머지 전부 → contype="0" conaffinity="0"  (Collision-Free)
"""
import xml.etree.ElementTree as ET

ET.register_namespace("", "")

KEEP_COLLISION_MESHES = {
    "L1_Wall1_1",
    "L1_Wall2_1",
    "L2_Wall3_1",
    "L9_PLATE_v3_1",
}

XML_PATH = r"C:\Crusher_isaacsim\MuJoCo_PlayGround\MJCF\Crusher_IsaacSim_colored.xml"

tree = ET.parse(XML_PATH)
root = tree.getroot()

changed = 0
kept    = 0

for geom in root.iter("geom"):
    geom_name = geom.get("name", "")
    mesh_name = geom.get("mesh", "")

    # ── ground geom은 항상 유지
    if geom_name == "ground":
        kept += 1
        continue

    # ── 유지 목록에 포함된 메시
    if mesh_name in KEEP_COLLISION_MESHES:
        kept += 1
        continue

    # ── CoACD hull 은 유지 (그룹 3)
    if mesh_name.startswith("L2_Left_Wall1_1_hull_"):
        kept += 1
        continue

    # ── 이미 collision-free 인 경우 (skip 중복 처리)
    if geom.get("contype") == "0" and geom.get("conaffinity") == "0":
        changed += 1  # 이미 처리된 것도 카운트
        continue

    # ── Collision-Free 적용
    geom.set("contype",    "0")
    geom.set("conaffinity","0")
    label = mesh_name or geom_name or "(unnamed)"
    print(f"  [CF] {label}")
    changed += 1

print(f"\n충돌 유지: {kept}개  /  Collision-Free 적용: {changed}개")

# ── 들여쓰기 보정 (Python 3.9+)
try:
    ET.indent(root, space="  ")
except AttributeError:
    pass  # 3.8 이하는 indent 없음

tree.write(XML_PATH, encoding="unicode", xml_declaration=True)
print(f"저장 완료: {XML_PATH}")

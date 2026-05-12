import xml.etree.ElementTree as ET

tree = ET.parse(r'C:\Temp\Crusher_IsaacSim.urdf')
root = tree.getroot()

# 두 번째 8_Link3_Shaft_1 링크를 8_Link3_Shaft_2로 변경
seen = False
for link in root.findall("link"):
    if link.attrib["name"] == "8_Link3_Shaft_1":
        if seen:
            link.set("name", "8_Link3_Shaft_2")
        else:
            seen = True

# Slider 56 조인트의 child도 변경
for joint in root.findall("joint"):
    if joint.attrib["name"] == "Slider 56":
        child = joint.find("child")
        child.set("link", "8_Link3_Shaft_2")

ET.indent(tree, space="  ")
tree.write(r"C:\Temp\Crusher_IsaacSim.urdf", xml_declaration=True, encoding="utf-8")
print("fixed and saved")

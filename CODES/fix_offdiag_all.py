import xml.etree.ElementTree as ET
import numpy as np

tree = ET.parse(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
root = tree.getroot()

fixed = 0
for link in root.findall("link"):
    inertia = link.find(".//inertia")
    if inertia is None: continue
    ixx=float(inertia.attrib["ixx"]); iyy=float(inertia.attrib["iyy"]); izz=float(inertia.attrib["izz"])
    ixy=float(inertia.attrib["ixy"]); iyz=float(inertia.attrib["iyz"]); ixz=float(inertia.attrib["ixz"])

    # off-diagonal? ?? ???? 10% ?? ? 0?? ???
    min_diag = min(ixx, iyy, izz)
    if abs(ixy) >= min_diag * 0.1 or abs(iyz) >= min_diag * 0.1 or abs(ixz) >= min_diag * 0.1:
        inertia.set("ixy", "0"); inertia.set("iyz", "0"); inertia.set("ixz", "0")
        fixed += 1
        print(f"fixed: {link.attrib['name']}")

ET.indent(tree, space="  ")
tree.write(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf", xml_declaration=True, encoding="utf-8")
print(f"? {fixed}? ?? ??")

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
    I = np.array([[ixx,ixy,ixz],[ixy,iyy,iyz],[ixz,iyz,izz]])
    eigs = np.linalg.eigvalsh(I)
    if any(e <= 0 for e in eigs):
        inertia.set("ixy","0"); inertia.set("iyz","0"); inertia.set("ixz","0")
        fixed += 1
        print(f"fixed: {link.attrib['name']} eigs={np.round(eigs,12)}")

ET.indent(tree, space="  ")
tree.write(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf", xml_declaration=True, encoding="utf-8")
print(f"? {fixed}? ?? ??")

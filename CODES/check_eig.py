import xml.etree.ElementTree as ET
import numpy as np

tree = ET.parse(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
root = tree.getroot()

for link in root.findall("link"):
    inertia = link.find(".//inertia")
    if inertia is None: continue
    ixx = float(inertia.attrib["ixx"])
    iyy = float(inertia.attrib["iyy"])
    izz = float(inertia.attrib["izz"])
    ixy = float(inertia.attrib["ixy"])
    iyz = float(inertia.attrib["iyz"])
    ixz = float(inertia.attrib["ixz"])
    I = np.array([[ixx, ixy, ixz],
                  [ixy, iyy, iyz],
                  [ixz, iyz, izz]])
    eigs = np.linalg.eigvalsh(I)
    if any(e <= 0 for e in eigs):
        print(f"INVALID: {link.attrib["name"]} eigenvalues={eigs}")
print("check done")

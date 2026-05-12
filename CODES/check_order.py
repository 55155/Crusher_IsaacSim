import xml.etree.ElementTree as ET

tree = ET.parse(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
root = tree.getroot()

# ?? ???? mesh ?? ??
for i, link in enumerate(root.findall("link")):
    name = link.attrib["name"]
    meshes = [m.attrib.get("filename") for m in link.iter("mesh")]
    print(f"[{i}] {name}: {meshes}")

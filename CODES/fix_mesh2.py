import xml.etree.ElementTree as ET

tree = ET.parse(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
root = tree.getroot()

for mesh in root.iter("mesh"):
    fname = mesh.attrib.get("filename", "")
    if fname.startswith("../meshes/"):
        mesh.set("filename", fname.replace("../meshes/", "meshes/"))

ET.indent(tree, space="  ")
tree.write(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf", xml_declaration=True, encoding="utf-8")
print("done")

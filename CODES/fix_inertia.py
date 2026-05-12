import xml.etree.ElementTree as ET

tree = ET.parse(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
root = tree.getroot()

epsilon = 1e-6
count = 0

for inertia in root.iter("inertia"):
    for attr in ["ixx", "iyy", "izz", "ixy", "iyz", "ixz"]:
        val = float(inertia.attrib.get(attr, "0"))
        if abs(val) < epsilon:
            inertia.set(attr, str(epsilon))
            count += 1

ET.indent(tree, space="  ")
tree.write(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf", xml_declaration=True, encoding="utf-8")
print(f"??? ?: {count}?")

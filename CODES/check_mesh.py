import xml.etree.ElementTree as ET, os

tree = ET.parse(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
root = tree.getroot()

mesh_dir = r"C:\Temp\Crusher_IsaacSim_description\meshes"
missing = []
found = []

for mesh in root.iter("mesh"):
    fname = mesh.attrib.get("filename", "")
    stl_name = fname.split("/")[-1]
    full_path = os.path.join(mesh_dir, stl_name)
    if os.path.exists(full_path):
        found.append(stl_name)
    else:
        missing.append(stl_name)

missing = list(set(missing))
print(f"??: {len(set(found))}?")
print(f"??: {len(missing)}?")
for m in sorted(missing):
    print("  MISSING:", m)

import struct, os, xml.etree.ElementTree as ET

def stl_volume(path):
    with open(path, 'rb') as f:
        f.read(80)
        tri_count = struct.unpack('<I', f.read(4))[0]
        volume = 0.0
        for _ in range(tri_count):
            f.read(12)
            verts = [struct.unpack('<3f', f.read(12)) for _ in range(3)]
            f.read(2)
            v0, v1, v2 = verts
            volume += (v0[0]*(v1[1]*v2[2]-v1[2]*v2[1])
                      -v0[1]*(v1[0]*v2[2]-v1[2]*v2[0])
                      +v0[2]*(v1[0]*v2[1]-v1[1]*v2[0])) / 6.0
    return abs(volume)

# ?? ??
PLA        = 1240.0   # kg/m? (?? PLA)
STAINLESS  = 8000.0   # kg/m? (?????)

targets = {
    '3_RackGear_1':    PLA,
    '2_Linear_bush_1': STAINLESS,
    '8_Link3_Shaft_1': STAINLESS,
}

tree = ET.parse(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
root = tree.getroot()
mesh_dir = r"C:\Temp\Crusher_IsaacSim_description\urdf"

for link in root.findall("link"):
    name = link.attrib["name"]
    if name not in targets:
        continue

    new_density = targets[name]
    inertial = link.find("inertial")
    old_mass = float(inertial.find("mass").attrib["value"])

    mesh = link.find(".//visual/geometry/mesh")
    stl_path = os.path.join(mesh_dir, os.path.basename(mesh.attrib["filename"]))
    vol_mm3 = stl_volume(stl_path)
    vol_m3 = vol_mm3 * (0.001**3)

    new_mass = new_density * vol_m3
    ratio = new_mass / old_mass

    inertial.find("mass").set("value", str(new_mass))

    inertia = inertial.find("inertia")
    for attr in ["ixx", "iyy", "izz", "ixy", "iyz", "ixz"]:
        old_val = float(inertia.attrib[attr])
        inertia.set(attr, str(old_val * ratio))

    print(f"{name}: {old_mass:.6f}kg -> {new_mass:.6f}kg (density={new_density})")

ET.indent(tree, space="  ")
tree.write(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf", xml_declaration=True, encoding="utf-8")
print("done")

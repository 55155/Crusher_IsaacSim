import struct, os, xml.etree.ElementTree as ET

def stl_volume(path):
    with open(path, 'rb') as f:
        f.read(80)
        tri_count = struct.unpack('<I', f.read(4))[0]
        volume = 0.0
        for _ in range(tri_count):
            f.read(12)  # normal
            verts = [struct.unpack('<3f', f.read(12)) for _ in range(3)]
            f.read(2)
            # signed volume of tetrahedron
            v0, v1, v2 = verts
            volume += (v0[0]*(v1[1]*v2[2]-v1[2]*v2[1])
                      -v0[1]*(v1[0]*v2[2]-v1[2]*v2[0])
                      +v0[2]*(v1[0]*v2[1]-v1[1]*v2[0])) / 6.0
    return abs(volume)  # mm^3

tree = ET.parse(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
root = tree.getroot()
mesh_dir = r"C:\Temp\Crusher_IsaacSim_description\urdf"

print(f"{'??':<30} {'??(kg)':>10} {'??(kg/m3)':>12} {'????':>10}")
print("-"*70)

for link in root.findall("link"):
    name = link.attrib["name"]
    inertial = link.find("inertial")
    if inertial is None: continue
    mass = float(inertial.find("mass").attrib["value"])
    mesh = link.find(".//visual/geometry/mesh")
    if mesh is None: continue
    stl = os.path.join(mesh_dir, os.path.basename(mesh.attrib["filename"]))
    if not os.path.exists(stl): continue
    vol_mm3 = stl_volume(stl)
    scale = 0.001  # STL? mm??, URDF? m??
    vol_m3 = vol_mm3 * (scale**3)
    if vol_m3 == 0: continue
    density = mass / vol_m3
    if density < 1000:   mat = "?????"
    elif density < 2000: mat = "????"
    elif density < 3000: mat = "????"
    elif density < 6000: mat = "??/?"
    else:                mat = "??/?"
    print(f"{name:<30} {mass:>10.4f} {density:>12.1f} {mat:>10}")

import struct, os

def check_stl(path):
    size = os.path.getsize(path)
    with open(path, 'rb') as f:
        header = f.read(80)
        tri_count = struct.unpack('<I', f.read(4))[0]
    expected_size = 80 + 4 + tri_count * 50
    return size, tri_count, expected_size, size == expected_size

meshes_dir = r'C:\Temp\Crusher_IsaacSim_description\urdf\meshes'
for stl in sorted(os.listdir(meshes_dir)):
    path = os.path.join(meshes_dir, stl)
    size, tri, expected, valid = check_stl(path)
    if not valid:
        print(f'INVALID: {stl} | size={size}, expected={expected}, triangles={tri}')
    else:
        print(f'OK: {stl} | triangles={tri}')

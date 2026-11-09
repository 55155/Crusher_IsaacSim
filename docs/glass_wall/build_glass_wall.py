"""강화유리 엔드월 + 머신비전 카메라 간이설계 3D 모델(GLB) 생성.
원본 URDF 부품은 그대로 두고, 랙피니언 샘플백 홀더 계열은 removed_ 로, 새 부품은 new_ 로 이름을 붙인다.
좌표: URDF world [mm], z-up. 실행: python build_glass_wall.py -> glass_wall.glb
"""
import os, xml.etree.ElementTree as ET, numpy as np, trimesh
from trimesh.transformations import euler_matrix
from trimesh.visual.material import PBRMaterial

D = r"C:\Crusher_isaacsim\Crusher_IsaacSim_description\urdf"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "glass_wall.glb")

REMOVED = {"L2_Wall3_1", "L1_Wall1_1", "L1_Wall2_1", "L2_Left_Wall1_1", "L3_RackGear_1", "L1_Guide1_1",
           "L1_Guide2_1", "L2_GearCase2_1", "L2_GearCase3_1", "L3_GearCase1_1", "L2__Motor2_Braket_1",
           "L3_Motor2_1", "L4_Motor2_Shaft_1", "L5_Coupler_1", "L6_DcutShaft_1", "L7_Holder_Bearing1_1",
           "L7_Holder_Bearing2_1", "L7_SpurGear_1"}

# ── 핵심 치수 [mm] ──
CX, CZ = -48.3, 49.4          # 엔드플레이트 중심 (x, z)
Y0 = 336.3                     # 기존 벽 앞면 = 유리 앞면 (스트로크 기하 유지)
T_G = 12.0                     # 강화유리 두께 (중앙 집중하중 최악 15.6 MPa, design_checklist §1)
T_GK = 1.0                     # 실리콘/PTFE 개스킷
T_BK = 12.0                    # 프레임 백킹 두께
GW, GH = 80.0, 70.0            # 유리 크기 (x, z)
AP = 56.0                      # 지지 개구부 (정사각)
FX0, FX1, FZ0, FZ1 = -119.8, 0.2, 9.4, 94.4   # 프레임 외곽
F_LENS = 35.0; SENSOR_W, SENSOR_H = 13.13, 8.76; FOV_H = 30.0   # 1" IMX183 20MP mono, 35 mm
WD = 92.0                      # 유리 앞면 -> 렌즈 앞면 (물체거리 115 - 주점 깊이 ~19 - 유리 광학두께 보정 4)

def mat(rgb, a=1.0, metal=0.0, rough=0.6, emis=None):
    return PBRMaterial(baseColorFactor=[*rgb, a], metallicFactor=metal, roughnessFactor=rough,
                       alphaMode="BLEND" if a < 1 else "OPAQUE", doubleSided=a < 1,
                       emissiveFactor=emis)

M = dict(
    keep=mat((0.55, 0.57, 0.60), metal=0.3, rough=0.5),
    keep_dark=mat((0.18, 0.18, 0.20), metal=0.3, rough=0.5),
    removed=mat((0.85, 0.25, 0.20), a=0.35, rough=0.8),
    glass=mat((0.55, 0.85, 0.95), a=0.28, rough=0.05),
    frame=mat((0.32, 0.36, 0.42), metal=0.7, rough=0.35),
    gasket=mat((0.95, 0.55, 0.15), rough=0.9),
    al=mat((0.75, 0.77, 0.80), metal=0.6, rough=0.4),
    black=mat((0.06, 0.06, 0.07), rough=0.5),
    shroud=mat((0.08, 0.08, 0.09), a=0.45, rough=0.9),
    light=mat((0.95, 0.95, 0.90), emis=[0.9, 0.9, 0.8]),
    bolt=mat((0.6, 0.6, 0.62), metal=0.9, rough=0.3),
)

def T(o):
    if o is None: return np.eye(4)
    xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
    m = euler_matrix(*rpy, "sxyz"); m[:3, 3] = xyz; return m

scene = trimesh.Scene()
def add(name, mesh, m):
    mesh = mesh.copy(); mesh.visual = trimesh.visual.TextureVisuals(material=m)
    scene.add_geometry(mesh, node_name=name, geom_name=name)

def box(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0])
    b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]); return b

def cyl_y(r, y0, y1, x=CX, z=CZ, r_in=None):
    if r_in: c = trimesh.creation.annulus(r_min=r_in, r_max=r, height=y1 - y0, sections=64)
    else:    c = trimesh.creation.cylinder(radius=r, height=y1 - y0, sections=64)
    c.apply_transform(euler_matrix(-np.pi / 2, 0, 0)); c.apply_translation([x, (y0 + y1) / 2, z]); return c

def ring_rect(ox0, ox1, oz0, oz1, ix0, ix1, iz0, iz1, y0, y1):
    return trimesh.util.concatenate([box(ox0, ix0, y0, y1, oz0, oz1), box(ix1, ox1, y0, y1, oz0, oz1),
                                     box(ix0, ix1, y0, y1, oz0, iz0), box(ix0, ix1, y0, y1, iz1, oz1)])

# ── 1. 원본 부품 ──
root = ET.parse(os.path.join(D, "Crusher_IsaacSim.urdf")).getroot()
par = {j.find("child").get("link"): (j.find("parent").get("link"), T(j.find("origin"))) for j in root.findall("joint")}
def W(l): return W(par[l][0]) @ par[l][1] if l in par else np.eye(4)
for L in root.findall("link"):
    n = L.get("name"); v = L.find("visual")
    if v is None: continue
    m = trimesh.load(os.path.join(D, v.find("geometry/mesh").get("filename")))
    m.apply_scale(0.001); m.apply_transform(W(n) @ T(v.find("origin"))); m.apply_scale(1000.0)
    if n in REMOVED: add("removed_" + n, m, M["removed"])
    else: add("keep_" + n, m, M["keep_dark"] if n in ("L9_PLATE_v3_1", "base_link") else M["keep"])

# ── 2. 강화유리 + 개스킷 + SUS304 프레임 ──
gx0, gx1, gz0, gz1 = CX - GW / 2, CX + GW / 2, CZ - GH / 2, CZ + GH / 2
ax0, ax1, az0, az1 = CX - AP / 2, CX + AP / 2, CZ - AP / 2, CZ + AP / 2
y_g1 = Y0 + T_G; y_k1 = y_g1 + T_GK; y_f1 = y_k1 + T_BK
add("new_glass_tempered_t12", box(gx0, gx1, Y0, y_g1, gz0, gz1), M["glass"])
add("new_gasket_t1", ring_rect(gx0, gx1, gz0, gz1, ax0, ax1, az0, az1, y_g1, y_k1), M["gasket"])
c = 0.2  # 유리-포켓 틈
add("new_frame_SUS304", trimesh.util.concatenate([
    ring_rect(FX0, FX1, FZ0, FZ1, gx0 - c, gx1 + c, gz0 - c, gz1 + c, Y0, y_k1),   # 유리 포켓
    ring_rect(FX0, FX1, FZ0, FZ1, ax0, ax1, az0, az1, y_k1, y_f1)]), M["frame"])  # 백킹 + 개구부

# ── 3. 리브(거싯) + 발판 + 볼트 ──
def gusset(x0, x1):
    tri = np.array([[y_f1, FZ0 + 5], [y_f1 + 40, FZ0 + 5], [y_f1, 79.4]])
    poly = trimesh.creation.extrude_polygon(__import__("shapely.geometry", fromlist=["Polygon"]).Polygon(tri), x1 - x0)
    poly.apply_transform(np.array([[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)); return poly
parts = []
for x0, x1 in ((FX0, FX0 + 8), (FX1 - 8, FX1)):
    parts += [gusset(x0, x1), box(x0, x1, y_f1, y_f1 + 40, FZ0, FZ0 + 5)]
add("new_gusset_SUS304", trimesh.util.concatenate(parts), M["frame"])
bolts = []
for xm in (FX0 + 4, FX1 - 4):
    for yb in (y_f1 + 14, y_f1 + 32):
        b = trimesh.creation.cylinder(radius=4.25, height=4, sections=24); b.apply_translation([xm, yb, FZ0 + 7]); bolts.append(b)
add("new_bolts_M5", trimesh.util.concatenate(bolts), M["bolt"])

# ── 4. 차광 후드 + 링조명(편광) + 렌즈 + 카메라 ──
y_lens0 = Y0 + WD
add("new_shroud_black", ring_rect(CX - 32, CX + 32, CZ - 32, CZ + 32, CX - 30.5, CX + 30.5, CZ - 30.5, CZ + 30.5,
                                  y_f1, y_lens0 - 8), M["shroud"])
add("new_ringlight_polarized", cyl_y(38, y_lens0 - 6, y_lens0 + 4, r_in=22), M["light"])
add("new_polarizer_filter", cyl_y(20.5, y_lens0 - 2, y_lens0, r_in=None), M["black"])
add("new_lens_35mm_Cmount", cyl_y(19.5, y_lens0, y_lens0 + 50), M["black"])
y_cam0 = y_lens0 + 50
add("new_camera_20MP_1in", box(CX - 14.5, CX + 14.5, y_cam0, y_cam0 + 30, CZ - 14.5, CZ + 14.5), M["black"])

# ── 5. 카메라 지지 (베이스 연장판 + 포커스 슬라이드 + 라이저) ──
yb1 = 400.3
add("new_base_extension_AL6061", trimesh.util.concatenate([
    box(CX - 30, CX + 30, yb1, 540, -0.6, 9.4),
    box(CX - 30, CX - 20, 370, 430, 9.4, 15.4), box(CX + 20, CX + 30, 370, 430, 9.4, 15.4)]), M["al"])
add("new_focus_slide", box(CX - 20, CX + 20, y_cam0 - 15, y_cam0 + 50, 9.4, 14.4), M["al"])
add("new_camera_riser", box(CX - 15, CX + 15, y_cam0, y_cam0 + 30, 14.4, CZ - 14.5), M["al"])

# ── 6. 시야(FOV) 사각뿔 (유리 앞면의 60x50 영역) ──
apex = np.array([CX, y_lens0, CZ]); hw, hh = FOV_H / 2, FOV_H * SENSOR_H / SENSOR_W / 2
corners = np.array([[CX - hw, Y0, CZ - hh], [CX + hw, Y0, CZ - hh], [CX + hw, Y0, CZ + hh], [CX - hw, Y0, CZ + hh]])
fov = trimesh.Trimesh(vertices=np.vstack([apex, corners]), faces=[[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 1]], process=False)
add("fov_pyramid", fov, mat((1.0, 0.85, 0.2), a=0.18))

scene.export(OUT)
print("saved", OUT, round(os.path.getsize(OUT) / 1e6, 2), "MB")
print("lens front y =", y_lens0, " camera back y =", y_cam0 + 30, " frame back y =", y_f1)

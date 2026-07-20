"""
pipeline_legacy.py — Legacy_vs_IPC 비교의 "Legacy(PBD+Rigid)" 팔.

pipeline_ipc.py(=full_workflow.py 복사본)와 최대한 같은 씬(Crusher/플레이트/
로봇 v2/슬롯 계산/IK/페이즈 시퀀스)을 쓰되, 다음 4가지만 바꾼다 — 이게 이
비교가 실제로 측정하려는 변수다:

  1. 커플러      : IPCCoupler        -> LegacyCoupler(rigid_pbd=True)
  2. 봉투 재질    : FEM.Cloth         -> PBD.Cloth
  3. 정제 재질    : FEM.Elastic(캡슐) -> Rigid(같은 캡슐 형상)
  4. 파지 방식    : 순수 마찰 접촉    -> weld(파티클을 그리퍼 링크에 고정)
                    — docs/DigitalTwin.md §9 조합1: PBD+진짜 마�치 파지는
                    핑거가 봉투에 닿는 즉시 폭발해서 폐기됐고, 6+1 파일 셋의
                    Crushing.py 가 이미 이 weld 우회로 검증되어 있다
                    (fix_particles_to_link). 그 패턴을 그대로 쓴다.

**봉투 형상**: 실측 Samplebag STL(Samplebag_seal_pouch3.stl) 대신 procedural
5-panel proxy(Crushing.py/ipc_grasp_bag_test.py 의 make_bag() 패턴)를 쓴다 —
PBD 는 앞/뒤 패널 간격이 2*particle_size(≈5.7mm) 보다 좁으면 파티클-파티클
충돌 제약이 즉시 위반돼 폭발하는데(§Crushing.py 주석), 실측 STL 의 실링부는
그보다 얇을 수 있어서다. 로컬 축 관례(X=폭, Y=높이, Z=두께)와 전체 bbox
치수(폭 64mm, 높이 90mm, 두께 6mm)는 pipeline_ipc.py 의 실측 봉투와 맞춰서,
같은 BAG_EULER=(90,0,90) 를 적용하면 월드 좌표계에서 동일하게 놓인다.
symmetric proxy 라 실측 봉투의 SEAL_LOCAL_X(비대칭 실링 보정)는 필요 없다
(=0).

**타이밍**: 각 커플러가 검증된 자기 dt 를 그대로 쓴다(PBD: dt=1e-3+substeps
=10, Crushing.py 검증값 / IPC: dt=5e-3, full_workflow.py 검증값) — 같은
스텝수를 강제하면 한쪽이 불안정해진다. 대신 **페이즈별 시뮬레이션 시간(초)**
을 pipeline_ipc.py 와 동일하게 맞추고(N_phase = round(duration_sec/dt)),
결과 JSON 에 sim_time_sec 과 wall_clock_sec 을 둘 다 기록해 "같은 물리
시간동안 실제로 얼마나 걸렸는가"를 비교한다.

결과: RESULT/results_legacy_<ts>.json — pipeline_ipc.py 와 동일 스키마.
"""
import os, sys, shutil, tempfile, json, time
import xml.etree.ElementTree as ET
from datetime import datetime
import numpy as np
import trimesh as tm

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
import paths

sys.path.insert(0, os.path.join(os.path.dirname(_r), "utills"))
from primitive_tablet_generator import _capsule_surface

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4_OVERVIEW = os.path.join(OUT_DIR, f"legacy_{_TS}_overview.mp4")
MP4_BAGCAM = os.path.join(OUT_DIR, f"legacy_{_TS}_bagcam.mp4")
RESULT_JSON = os.path.join(OUT_DIR, f"results_legacy_{_TS}.json")

# ── Crusher + plate (pipeline_ipc.py 동일) ──────────────────────────────────
CRUSHER_SRC_XML = paths.MJCF_MAIN
CRUSHER_POS = (0.0, 0.0, 0.0)
CRUSHER_EULER = (0.0, 0.0, 90.0)
PLATE_PATH = paths.ALUMINUM_PLATE
PLATE_POSITIONS = [(0.5, -0.5, 0), (0.5, 0.5, 0), (-0.5, -0.5, 0), (-0.5, 0.5, 0)]

WALL_GEOMS_TO_ENABLE = {"base_link", "L1_Wall1_1", "L1_Wall2_1", "L2_Wall3_1"}
L7_LINK3_COM = "0.006 0 -0.005"

WALL_BACK_MESH = "L2_Wall3_1"
WALL_LEFT_MESH = "L2_Left_Wall1_1"
LEFTWALL_BODY_POS = (-0.017802, 0.286278, 0.016542)
LEFTWALL_GEOM_POS = (-0.286278, -0.016542, 0.017802)
_R_GEOM_HALF = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])

CRANK_JOINT = "L3_Bevel_GearBox_1_L4_Shaft_1"
WALL_JOINT = "L1_Guide1_1_L2_Left_Wall1_1"
CRANK_START_Q = -np.pi
WALL_OFFSET = 0.006
CLAMP_TARGET = -0.005
CRANK_KP, CRANK_KV = 2000.0, 100.0
WALL_KP, WALL_KV = 5000.0, 500.0


def patch_crusher_mjcf(src, dst, eq_solref="0.0002 50", eq_solimp="0.999 0.99999 1e-5"):
    tree = ET.parse(src); root = tree.getroot()
    eq = root.find("equality")
    if eq is not None:
        for j in list(eq.findall("joint")):
            eq.remove(j)
        for w in eq.findall("weld"):
            w.set("solref", eq_solref)
            w.set("solimp", eq_solimp)
    wb = root.find("worldbody")
    if wb is not None:
        for g in list(wb.findall("geom")):
            if g.get("name") == "ground":
                wb.remove(g)
        for g in wb.iter("geom"):
            if g.get("mesh") in WALL_GEOMS_TO_ENABLE:
                g.attrib.pop("contype", None)
                g.attrib.pop("conaffinity", None)
        for body in wb.iter("body"):
            if body.get("name") == "L7_Link3_1":
                inertial = body.find("inertial")
                if inertial is not None:
                    inertial.set("pos", L7_LINK3_COM)
    tree.write(dst)


def _prepare_crusher_mjcf():
    tmp_dir = tempfile.mkdtemp(prefix="crusher_mjcf_legacy_")
    src_dir = os.path.dirname(CRUSHER_SRC_XML)
    for f in os.listdir(src_dir):
        s = os.path.join(src_dir, f)
        if os.path.isfile(s):
            shutil.copy2(s, os.path.join(tmp_dir, f))
    dst = os.path.join(tmp_dir, "Crusher_genesis.xml")
    patch_crusher_mjcf(CRUSHER_SRC_XML, dst)
    return dst


def crusher_mesh_world_aabb(mesh_name, body_pos=(0., 0., 0.), geom_pos=(0., 0., 0.)):
    yaw = np.radians(CRUSHER_EULER[2])
    R_e = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                    [np.sin(yaw), np.cos(yaw), 0.],
                    [0., 0., 1.]])
    v = tm.load(os.path.join(paths.MJCF_DIR, f"{mesh_name}.stl")).vertices * 0.001
    local = np.asarray(geom_pos) + v
    in_crusher = np.asarray(body_pos) + (_R_GEOM_HALF @ local.T).T
    w = np.array(CRUSHER_POS) + (R_e @ in_crusher.T).T
    return w.min(axis=0), w.max(axis=0)


# ── M0609+RG2(v2) + 정제(Rigid) + 샘플백(PBD proxy) ─────────────────────────
ROBOT_MJCF = os.path.join(paths.ROBOTS_DIR, "m0609_rg2_v2.xml")
COACD_DIR_REL = "rg2/reference_onrobot_ros/meshes/rg2_v1/coacd"
FLEX_FINGER_HULLS = [f"flex_finger_hull_{i:03d}.stl" for i in range(7)]

# 봉투 procedural proxy 치수 — pipeline_ipc.py 실측 봉투 bbox 와 맞춤(폭64/높이90/두께6mm)
BAG_W, BAG_H, BAG_D = 0.064, 0.090, 0.006
BAG_NW, BAG_NH, BAG_ND = 8, 11, 2
BAG_STL_PATH = os.path.join(OUT_DIR, "_bag_pbd_proxy.stl")
PARTICLE_SIZE = 2.83e-3     # Crushing.py 검증값 (2*PARTICLE_SIZE≈5.7mm < D=6mm 안전)
STRETCH_COMPLIANCE = 1e-3
BENDING_COMPLIANCE = 1e-3

CAP_RADIUS_MM, CAP_CYL_H_MM = 2.0, 1.0
TABLET_RHO = 1300.0
TABLET_FRICTION = 0.5
TABLET_STL_PATH = os.path.join(OUT_DIR, "_tablet_rigid_capsule.stl")

DT, SUBSTEPS = 1e-3, 10   # Crushing.py 검증값(PBD 안정성)

FINGER_LINKS = ("f1_flex_finger", "f2_flex_finger")
Q_GRASP = np.array([0, -0.40, 1.30, 0, 2.00, 0.0], float)
Q_LIFT = np.array([0, -0.11, 0.60, 0, 2.41, 0.0], float)
FING_OPEN, FING_CLOSE = 1.00, 1.20

ROBOT_OFFSET = np.array([-0.330, -0.65, 0.0])
FINGER_MID_BASE = np.array([0.20339339, 0.00618061, 0.43607193])
FINGER_MID = FINGER_MID_BASE + ROBOT_OFFSET

BAG_SCALE = 1.0
BAG_EULER = (90, 0, 90)
BAG_HALF_H = BAG_H / 2.0
TOP_GRIP_MARGIN = 0.008
# symmetric proxy 라 SEAL_LOCAL_X 보정 불필요(실측 봉투는 -0.028 오프셋 사용)
BAG_POS = (FINGER_MID[0], FINGER_MID[1], FINGER_MID[2] - BAG_HALF_H + TOP_GRIP_MARGIN)

SHELF_TOP = BAG_POS[2] - BAG_HALF_H - 0.0015
SHELF_SIZE = (0.10, 0.10, 0.02)
SHELF_POS = (BAG_POS[0], BAG_POS[1], SHELF_TOP - SHELF_SIZE[2] / 2)

BAG_MOUTH_Z = BAG_POS[2] + BAG_HALF_H
TABLET_DROP_H = 0.015
TABLET_POS = (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z + TABLET_DROP_H)

# ── 페이즈 duration(초) — pipeline_ipc.py(N_i * DT=5e-3) 와 동일하게 맞추고,
#    각자 자기 dt 로 스텝수를 재환산한다(각 솔버 고유의 안정 dt 유지).
PHASE_DURATION_SEC = dict(
    prep=1.00, drop=0.75, settle=0.30, close=0.40, grasp=0.20,
    lift=1.00, hold=0.50, above=2.00, insert=2.00, settle2=0.50,
    clamp=2.00, release=0.50,
)
def _n(name):
    return max(1, round(PHASE_DURATION_SEC[name] / DT))
N_PREP, N_DROP, N_SETTLE, N_CLOSE, N_GRASP = _n("prep"), _n("drop"), _n("settle"), _n("close"), _n("grasp")
N_LIFT, N_HOLD, N_ABOVE, N_INSERT, N_SETTLE2 = _n("lift"), _n("hold"), _n("above"), _n("insert"), _n("settle2")
N_CLAMP, N_RELEASE = _n("clamp"), _n("release")

OVERVIEW_CAM_POS = (0.9, -1.7, 1.3)
OVERVIEW_CAM_LOOK = (-0.15, -0.35, 0.15)
BAGCAM_OFFSET = np.array([0.20, -0.20, 0.12])


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def _pos_of(entity):
    x = _npy(entity.get_particles_pos())
    return x[0] if x.ndim == 3 else x


def _prepare_robot_mjcf():
    src_dir = paths.ROBOTS_DIR
    tmp_dir = tempfile.mkdtemp(prefix="m0609_legacy_v2_")
    for root_dir, _, files in os.walk(src_dir):
        rel = os.path.relpath(root_dir, src_dir)
        dst_dir = os.path.join(tmp_dir, rel) if rel != "." else tmp_dir
        os.makedirs(dst_dir, exist_ok=True)
        for f in files:
            shutil.copy2(os.path.join(root_dir, f), os.path.join(dst_dir, f))
    dst = os.path.join(tmp_dir, "m0609_rg2_v2_patched.xml")
    tree = ET.parse(ROBOT_MJCF)
    root = tree.getroot()

    asset = root.find("asset")
    for i, hull_file in enumerate(FLEX_FINGER_HULLS):
        mesh_el = ET.SubElement(asset, "mesh")
        mesh_el.set("name", f"flex_finger_hull_{i:03d}")
        mesh_el.set("file", f"{COACD_DIR_REL}/{hull_file}")

    wb = root.find("worldbody")
    for j in wb.iter("joint"):
        j.attrib.pop("damping", None)
        j.attrib.pop("frictionloss", None)
    for body in wb.iter("body"):
        if body.get("name") in ("f1_flex_finger", "f2_flex_finger"):
            for i in range(len(FLEX_FINGER_HULLS)):
                g = ET.SubElement(body, "geom")
                g.set("type", "mesh")
                g.set("mesh", f"flex_finger_hull_{i:03d}")
                g.set("contype", "1")
                g.set("conaffinity", "1")
                g.set("group", "0")
                g.set("friction", "1.5 0.02 0.001")

    for tag in ("actuator", "equality"):
        el = root.find(tag)
        if el is not None:
            root.remove(el)
    tree.write(dst)
    return dst


def _panel(fn, nu, nv):
    t = []
    for i in range(nu):
        for j in range(nv):
            a, b = fn(i / nu, j / nv), fn((i + 1) / nu, j / nv)
            c, d = fn((i + 1) / nu, (j + 1) / nv), fn(i / nu, (j + 1) / nv)
            t += [[a, b, c], [a, c, d]]
    return t


def make_bag_proxy():
    """5-panel open pouch(front+back+bottom+left+right) — Crushing.py/
    ipc_grasp_bag_test.py 의 make_bag() 과 동일 패턴. 로컬 X=폭,Y=높이,Z=두께."""
    tris = []
    tris += _panel(lambda u, v: np.array([u * BAG_W, v * BAG_H, 0.0]), BAG_NW, BAG_NH)
    tris += _panel(lambda u, v: np.array([u * BAG_W, v * BAG_H, BAG_D]), BAG_NW, BAG_NH)
    tris += _panel(lambda u, v: np.array([u * BAG_W, 0.0, v * BAG_D]), BAG_NW, BAG_ND)
    tris += _panel(lambda u, v: np.array([0.0, u * BAG_H, v * BAG_D]), BAG_NH, BAG_ND)
    tris += _panel(lambda u, v: np.array([BAG_W, u * BAG_H, v * BAG_D]), BAG_NH, BAG_ND)
    v = np.array([p for t in tris for p in t])
    f = np.arange(len(v)).reshape(-1, 3)
    m = tm.Trimesh(vertices=v, faces=f, process=False)
    m.merge_vertices(digits_vertex=7)
    m.vertices -= m.bounding_box.centroid
    m.export(BAG_STL_PATH)
    print(f"[bag-proxy] {len(m.vertices)} verts, {len(m.faces)} faces -> {BAG_STL_PATH}")


def make_tablet_rigid_stl():
    """FEM 팔의 make_capsule_tets_v2 와 같은 캡슐 표면(반지름/높이 동일)을
    Rigid mesh 로 그대로 내보낸다 — 형상을 맞춰야 공정한 비교가 된다."""
    surf_v, surf_f = _capsule_surface(CAP_RADIUS_MM, CAP_CYL_H_MM, n_theta=12, n_cap_rings=4)
    tm.Trimesh(vertices=surf_v, faces=surf_f, process=False).export(TABLET_STL_PATH)
    print(f"[tablet-proxy] {len(surf_v)} verts -> {TABLET_STL_PATH}")


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" [Legacy] Legacy_vs_IPC pipeline_legacy  (viewer={use_viewer})")
    print("=" * 60)

    t_wall0 = time.perf_counter()
    phase_times = {}

    crusher_xml = _prepare_crusher_mjcf()
    robot_xml = _prepare_robot_mjcf()
    make_bag_proxy()
    make_tablet_rigid_stl()

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, -9.81)),
        pbd_options=gs.options.PBDOptions(max_density_solver_iterations=2, particle_size=PARTICLE_SIZE),
        coupler_options=gs.options.LegacyCouplerOptions(rigid_pbd=True),
        vis_options=gs.options.VisOptions(
            background_color=(0.93, 0.94, 0.96),
            ambient_light=(0.16, 0.16, 0.18),
            lights=[
                {"type": "directional", "dir": (-1, -1, -1), "color": (1.0, 1.0, 1.0), "intensity": 6.0},
                {"type": "directional", "dir": (1, 1, -0.6), "color": (1.0, 1.0, 1.0), "intensity": 1.2},
            ],
        ),
        show_viewer=use_viewer,
    )

    for p in PLATE_POSITIONS:
        scene.add_entity(
            gs.morphs.Mesh(file=PLATE_PATH, fixed=True, pos=p),
            material=gs.materials.Rigid(),
            surface=gs.surfaces.Default(color=(0.82, 0.82, 0.85), metallic=0.85, roughness=0.3),
        )

    # Crusher — Crushing.py 검증 설정(decimate/convexify 끔, 원본 STL 보존) 재사용.
    crusher = scene.add_entity(
        gs.morphs.MJCF(file=crusher_xml, pos=CRUSHER_POS, euler=CRUSHER_EULER,
                       decimate=False, convexify=False),
        surface=gs.surfaces.Default(smooth=False),
    )

    scene.add_entity(gs.morphs.Plane(visualization=False), material=gs.materials.Rigid())
    scene.add_entity(
        gs.morphs.Box(size=SHELF_SIZE, pos=SHELF_POS, fixed=True),
        material=gs.materials.Rigid(),
        surface=gs.surfaces.Default(color=(0.75, 0.78, 0.82)),
    )

    robot = scene.add_entity(
        gs.morphs.MJCF(file=robot_xml, pos=tuple(ROBOT_OFFSET), decimate=False),
        material=gs.materials.Rigid(),
    )

    bag = scene.add_entity(
        material=gs.materials.PBD.Cloth(
            stretch_compliance=STRETCH_COMPLIANCE,
            bending_compliance=BENDING_COMPLIANCE,
        ),
        morph=gs.morphs.Mesh(file=BAG_STL_PATH, scale=BAG_SCALE, pos=BAG_POS, euler=BAG_EULER),
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.7, roughness=0.9, double_sided=True),
    )

    tablet = scene.add_entity(
        material=gs.materials.Rigid(rho=TABLET_RHO, friction=TABLET_FRICTION),
        morph=gs.morphs.Mesh(file=TABLET_STL_PATH, scale=1e-3, pos=TABLET_POS, fixed=False),
        surface=gs.surfaces.Default(color=(0.9, 0.9, 0.85), roughness=0.6),
    )

    cam_over = scene.add_camera(res=(1280, 960), pos=OVERVIEW_CAM_POS, lookat=OVERVIEW_CAM_LOOK,
                                fov=48, GUI=False)
    cam_bag = scene.add_camera(res=(960, 720), pos=tuple(np.array(BAG_POS) + BAGCAM_OFFSET),
                               lookat=BAG_POS, fov=40, GUI=False)

    print("\n[build] scene.build() 시작...")
    t_build0 = time.perf_counter()
    scene.build(n_envs=0)
    phase_times["build"] = time.perf_counter() - t_build0
    print(f"[build] 성공 ({phase_times['build']:.1f}s)")

    # ── 봉투 형상 고정(바닥+양측면, 입구는 자유) — fix_particles (link 없음) ──
    bag_pos0 = _pos_of(bag)
    by, bz = bag_pos0[:, 1], bag_pos0[:, 2]
    bag_bottom_mask = bz < bz.min() + 0.012
    bag_side_mask = (by < by.min() + 0.008) | (by > by.max() - 0.008)
    bag_fixed_idx = np.where(bag_bottom_mask | bag_side_mask)[0].astype(int).tolist()
    bag.fix_particles(particles_idx_local=bag_fixed_idx)
    print(f"[bag] shape 고정: {len(bag_fixed_idx)}/{len(bz)} 정점(바닥+양측면), 입구는 자유")

    wb_lo, wb_hi = crusher_mesh_world_aabb(WALL_BACK_MESH)
    wl_lo, wl_hi = crusher_mesh_world_aabb(WALL_LEFT_MESH, LEFTWALL_BODY_POS, LEFTWALL_GEOM_POS)
    gap_lo_x, gap_hi_x = sorted([wb_hi[0], wl_lo[0]])
    gap_cx = (gap_lo_x + gap_hi_x) / 2.0
    gap_width = gap_hi_x - gap_lo_x
    y_lo = max(wb_lo[1], wl_lo[1]); y_hi = min(wb_hi[1], wl_hi[1])
    gap_cy = (y_lo + y_hi) / 2.0
    wall_top_z = max(wb_hi[2], wl_hi[2])
    w1_lo, w1_hi = crusher_mesh_world_aabb("L1_Wall1_1")
    pocket_cx = (w1_lo[0] + w1_hi[0]) / 2.0
    pocket_cy = (w1_lo[1] + w1_hi[1]) / 2.0
    wall_center_z = (wall_top_z + wb_lo[2]) / 2.0
    print(f"[slot] gap_cx={gap_cx:.4f} gap_cy={gap_cy:.4f} gap_width={gap_width*1000:.1f}mm wall_top_z={wall_top_z:.4f}")
    print(f"[slot] pocket(L1_Wall1_1) center=({pocket_cx:.4f},{pocket_cy:.4f})  wall_center_z={wall_center_z:.4f}")
    SLOT_TARGET = np.array([gap_cx, gap_cy, wall_center_z])

    crusher_joints = {j.name: j for j in crusher.joints if j.name}
    def _scalar_dof(name):
        d = crusher_joints[name].dofs_idx_local
        return d[0] if isinstance(d, (list, tuple, np.ndarray)) else d
    crank_dof = _scalar_dof(CRANK_JOINT)
    wall_dof = _scalar_dof(WALL_JOINT)
    crusher.set_dofs_kp(np.array([CRANK_KP]), dofs_idx_local=[crank_dof])
    crusher.set_dofs_kv(np.array([CRANK_KV]), dofs_idx_local=[crank_dof])
    crusher.set_dofs_kp(np.array([WALL_KP]), dofs_idx_local=[wall_dof])
    crusher.set_dofs_kv(np.array([WALL_KV]), dofs_idx_local=[wall_dof])

    left_link = robot.get_link(FINGER_LINKS[0])
    grip_link_idx = left_link.idx

    q_grasp, q_lift = Q_GRASP, Q_LIFT
    robot.set_dofs_position(np.concatenate([q_grasp, [FING_OPEN] * 6]))

    cam_over.start_recording()
    cam_bag.start_recording()

    def _bag_com():
        p = _pos_of(bag)
        return np.nanmean(p, axis=0)

    def _tablet_z():
        p = _npy(tablet.get_pos()).squeeze()
        return float(p[2]) if p.ndim == 1 else float(p[:, 2].mean())

    def _finger_z():
        return float(_npy(left_link.get_pos()).squeeze()[2])

    def render_cams():
        cam_over.render()
        bc = _bag_com()
        cam_bag.set_pose(pos=tuple(bc + BAGCAM_OFFSET), lookat=tuple(bc), up=(0, 0, 1))
        cam_bag.render()

    grip_idx_holder = {"idx": np.array([], dtype=int)}

    def run_arm(name, q0, q1, f0, f1, n, crank_q=None, wall_q=None, trace=False,
                attach=False, release_fixed=False, release_grip=False):
        if attach:
            cur = _pos_of(bag)
            d_to_mid = np.linalg.norm(cur - FINGER_MID, axis=1)
            grip_idx_holder["idx"] = np.where(d_to_mid < 0.020)[0].astype(int)
            bag.fix_particles_to_link(link_idx=grip_link_idx, particles_idx_local=grip_idx_holder["idx"].tolist())
            print(f"[grasp] attach {len(grip_idx_holder['idx'])} particles -> {FINGER_LINKS[0]}")
        if release_fixed:
            bag.release_particle(particles_idx_local=bag_fixed_idx)
            print("[bag] shape 고정 해제 — 이제부터 weld 파지만 유지")
        if release_grip:
            bag.release_particle(particles_idx_local=grip_idx_holder["idx"].tolist())
            print("[release] weld 해제")
        t0 = time.perf_counter()
        for k in range(n):
            s = (k + 1) / n
            q = q0 + (q1 - q0) * s
            f = f0 + (f1 - f0) * s
            robot.set_dofs_position(np.concatenate([q, [f] * 6]))
            if crank_q is not None:
                crusher.control_dofs_position(np.array([crank_q]), dofs_idx_local=[crank_dof])
            if wall_q is not None:
                crusher.control_dofs_position(np.array([wall_q]), dofs_idx_local=[wall_dof])
            scene.step()
            render_cams()
            if trace and k % (max(1, n // 4)) == 0:
                print(f"    [{name} k={k:4d}] tablet_z={_tablet_z()*1e3:+.2f}mm bag_com={_bag_com()}")
        phase_times[name] = time.perf_counter() - t0
        bc = _bag_com()
        print(f"[phase] {name:8s} @done  bag_com={bc}  finger_z={_finger_z():.4f}  "
              f"tablet_z={_tablet_z()*1e3:+.2f}mm  ({phase_times[name]:.1f}s wall)")

    print(f"\n[phase] 0 prep ({N_PREP*DT:.2f}s sim) — 크랭크 0->{CRANK_START_Q:+.3f}rad, "
          f"Left_Wall 0->{WALL_OFFSET*1000:+.0f}mm")
    t0 = time.perf_counter()
    for k in range(N_PREP):
        s = (k + 1) / N_PREP
        crusher.control_dofs_position(np.array([CRANK_START_Q * s]), dofs_idx_local=[crank_dof])
        crusher.control_dofs_position(np.array([WALL_OFFSET * s]), dofs_idx_local=[wall_dof])
        robot.set_dofs_position(np.concatenate([q_grasp, [FING_OPEN] * 6]))
        scene.step()
        render_cams()
    phase_times["prep"] = time.perf_counter() - t0
    print(f"[phase] prep     @done  ({phase_times['prep']:.1f}s wall)")

    run_arm("drop", q_grasp, q_grasp, FING_OPEN, FING_OPEN, N_DROP,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET, trace=True)
    run_arm("settle", q_grasp, q_grasp, FING_OPEN, FING_OPEN, N_SETTLE,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET, release_fixed=True)

    run_arm("close", q_grasp, q_grasp, FING_OPEN, FING_CLOSE, N_CLOSE,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)
    run_arm("grasp", q_grasp, q_grasp, FING_CLOSE, FING_CLOSE, N_GRASP,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET, attach=True)
    run_arm("lift", q_grasp, q_lift, FING_CLOSE, FING_CLOSE, N_LIFT,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)
    run_arm("hold", q_lift, q_lift, FING_CLOSE, FING_CLOSE, N_HOLD,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)

    q_insert_quat = _npy(left_link.get_quat()).squeeze()
    above_z = wall_top_z + 0.20
    INSERT_MARGIN_ABOVE_CENTER = 0.052
    insert_z = wall_center_z + INSERT_MARGIN_ABOVE_CENTER
    target_xy = np.array([gap_cx, gap_cy])
    print(f"[slot] wall_center_z={wall_center_z:.4f}  insert_z(finger)={insert_z:.4f}  "
          f"margin={INSERT_MARGIN_ABOVE_CENTER*1000:.0f}mm")

    target_above = np.array([target_xy[0], target_xy[1], above_z])
    qpos_above = _npy(robot.inverse_kinematics(
        link=left_link, pos=target_above, quat=q_insert_quat, dofs_idx_local=np.arange(6)))[:6]
    print(f"\n[ik] above-slot target={target_above}  arm_q={qpos_above}")
    run_arm("above", q_lift, qpos_above, FING_CLOSE, FING_CLOSE, N_ABOVE,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET, trace=True)

    target_insert = np.array([target_xy[0], target_xy[1], insert_z])
    qpos_insert = _npy(robot.inverse_kinematics(
        link=left_link, pos=target_insert, quat=q_insert_quat, dofs_idx_local=np.arange(6)))[:6]
    print(f"[ik] insert target={target_insert}  arm_q={qpos_insert}")
    run_arm("insert", qpos_above, qpos_insert, FING_CLOSE, FING_CLOSE, N_INSERT,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET, trace=True)
    run_arm("settle2", qpos_insert, qpos_insert, FING_CLOSE, FING_CLOSE, N_SETTLE2,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)

    print(f"\n[phase] 9 clamp ({N_CLAMP*DT:.2f}s sim) — Left_Wall {WALL_OFFSET*1000:+.1f}mm -> "
          f"{CLAMP_TARGET*1000:+.1f}mm")
    t0 = time.perf_counter()
    for k in range(N_CLAMP):
        s = (k + 1) / N_CLAMP
        wq = WALL_OFFSET + (CLAMP_TARGET - WALL_OFFSET) * s
        crusher.control_dofs_position(np.array([wq]), dofs_idx_local=[wall_dof])
        crusher.control_dofs_position(np.array([CRANK_START_Q]), dofs_idx_local=[crank_dof])
        robot.set_dofs_position(np.concatenate([qpos_insert, [FING_CLOSE] * 6]))
        scene.step()
        render_cams()
    phase_times["clamp"] = time.perf_counter() - t0
    bag_com_clamp = _bag_com()
    dist_clamp = float(np.linalg.norm(bag_com_clamp - SLOT_TARGET))
    dist_clamp_xy = float(np.linalg.norm(bag_com_clamp[:2] - SLOT_TARGET[:2]))
    print(f"[phase] clamp    @done  bag_com={bag_com_clamp}  ({phase_times['clamp']:.1f}s wall)  "
          f"dist_to_slot={dist_clamp*1000:.1f}mm (xy={dist_clamp_xy*1000:.1f}mm)")

    run_arm("release", qpos_insert, qpos_insert, FING_CLOSE, FING_OPEN, N_RELEASE,
            crank_q=CRANK_START_Q, wall_q=CLAMP_TARGET, trace=True, release_grip=True)

    bag_com_final = _bag_com()
    dist_final = float(np.linalg.norm(bag_com_final - SLOT_TARGET))
    dist_final_xy = float(np.linalg.norm(bag_com_final[:2] - SLOT_TARGET[:2]))

    cam_over.stop_recording(save_to_filename=MP4_OVERVIEW, fps=30)
    cam_bag.stop_recording(save_to_filename=MP4_BAGCAM, fps=30)
    print(f"\n[saved] overview -> {MP4_OVERVIEW}")
    print(f"[saved] bagcam   -> {MP4_BAGCAM}")

    total_wall = time.perf_counter() - t_wall0
    total_steps = N_PREP + N_DROP + N_SETTLE + N_CLOSE + N_GRASP + N_LIFT + N_HOLD + \
                  N_ABOVE + N_INSERT + N_SETTLE2 + N_CLAMP + N_RELEASE
    sim_time = total_steps * DT
    results = {
        "combo": "legacy",
        "coupler": "LegacyCoupler(rigid_pbd=True)",
        "bag_material": "PBD.Cloth",
        "tablet_material": "Rigid",
        "grasp_method": "weld (fix_particles_to_link cheat)",
        "dt": DT,
        "substeps": SUBSTEPS,
        "total_steps": total_steps,
        "sim_time_sec": sim_time,
        "wall_clock_sec": total_wall,
        "steps_per_sec": total_steps / total_wall,
        "phase_times_sec": phase_times,
        "slot_target": SLOT_TARGET.tolist(),
        "bag_com_at_clamp_end": bag_com_clamp.tolist(),
        "dist_to_slot_at_clamp_end_mm": dist_clamp * 1000,
        "dist_to_slot_xy_at_clamp_end_mm": dist_clamp_xy * 1000,
        "bag_com_final": bag_com_final.tolist(),
        "dist_to_slot_final_mm": dist_final * 1000,
        "dist_to_slot_xy_final_mm": dist_final_xy * 1000,
    }
    with open(RESULT_JSON, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2, ensure_ascii=False)
    print(f"\n[saved] {RESULT_JSON}")
    print(f"[summary] total_wall={total_wall:.1f}s  sim_time={sim_time:.2f}s  "
          f"dist_to_slot(final)={dist_final*1000:.1f}mm (xy={dist_final_xy*1000:.1f}mm)")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

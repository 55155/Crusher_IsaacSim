"""
pipeline_ipc.py — Legacy_vs_IPC 비교의 "IPC 팔".

Crusher_M0609_RG2_Tablet_Samplebag/full_workflow.py 를 그대로 복사해온
것(2026-07-16 버전, 6+1 파일 셋 밖의 기존 검증된 FEM+IPC 파이프라인)에 계측만
추가했다: 페이즈별/총 wall-clock 시간, 그리고 clamp 종료·release 종료 시점의
봉투 COM ↔ 슬롯 목표점 거리. pipeline_legacy.py(PBD.Cloth 봉투 + Rigid 정제 +
LegacyCoupler(rigid_pbd) + weld 파지)와 같은 스키마의 results_ipc.json 을 낸다.

바뀐 점(원본 full_workflow.py 대비): OUT_DIR 이 이 디렉토리의 RESULT/, 타이머
+ JSON 출력 추가. 시뮬레이션 로직 자체(재질/커플러/페이즈/스텝수/슬롯 계산)는
전혀 건드리지 않았다 — "같은 일을 하는 IPC 코드"를 그대로 옮겨와야 비교가
의미 있기 때문.
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
from primitive_tablet_generator import make_capsule_tets_v2, add_analytic_fem_entity
from fem_ipc_workarounds import patch_fem_vertex_constraints

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4_OVERVIEW = os.path.join(OUT_DIR, f"ipc_{_TS}_overview.mp4")
MP4_BAGCAM = os.path.join(OUT_DIR, f"ipc_{_TS}_bagcam.mp4")
RESULT_JSON = os.path.join(OUT_DIR, f"results_ipc_{_TS}.json")

# ── Crusher + plate (full_workflow.py 동일) ─────────────────────────────────
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
    tmp_dir = tempfile.mkdtemp(prefix="crusher_mjcf_ipc_")
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


# ── M0609+RG2(v2) + 정제 + 샘플백 (full_workflow.py 동일) ───────────────────
ROBOT_MJCF = os.path.join(paths.ROBOTS_DIR, "m0609_rg2_v2.xml")
COACD_DIR_REL = "rg2/reference_onrobot_ros/meshes/rg2_v1/coacd"
FLEX_FINGER_HULLS = [f"flex_finger_hull_{i:03d}.stl" for i in range(7)]
BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag", "Samplebag_seal_pouch3.stl")
BAG_PANEL_HALF_W, BAG_PANEL_HALF_H = 0.032, 0.045
SEAL_BAND_WIDTH = 0.010
BAG_BODY_COLOR = (247, 247, 242)
SEAL_COLOR = (200, 60, 40)

CAP_RADIUS_MM, CAP_CYL_H_MM = 2.0, 1.0
TABLET_E, TABLET_NU, TABLET_RHO = 5.0e4, 0.45, 1300.0
TABLET_FRICTION = 0.5

DT = 5e-3
IPC_D_HAT = 1.0e-4

CLOTH_E, CLOTH_NU, CLOTH_RHO = 4.0e5, 0.499, 200.0
CLOTH_THICK, CLOTH_BEND = 1.0e-3, 400.0
CLOTH_FRICTION = 0.8
FEM_DAMPING = 0.2

FINGER_LINKS = ("f1_flex_finger", "f2_flex_finger")
Q_GRASP = np.array([0, -0.40, 1.30, 0, 2.00, 0.0], float)
Q_LIFT = np.array([0, -0.11, 0.60, 0, 2.41, 0.0], float)
FING_OPEN, FING_CLOSE = 1.00, 1.20

ROBOT_OFFSET = np.array([-0.330, -0.65, 0.0])
FINGER_MID_BASE = np.array([0.20339339, 0.00618061, 0.43607193])
FINGER_MID = FINGER_MID_BASE + ROBOT_OFFSET

BAG_SCALE = 1.0
BAG_EULER = (90, 0, 90)
SEAL_LOCAL_X = -0.028
BAG_HALF_H = 0.045
TOP_GRIP_MARGIN = 0.008
BAG_POS = (FINGER_MID[0], FINGER_MID[1] - SEAL_LOCAL_X,
           FINGER_MID[2] - BAG_HALF_H + TOP_GRIP_MARGIN)

SHELF_TOP = BAG_POS[2] - BAG_HALF_H - 0.0015
SHELF_SIZE = (0.10, 0.10, 0.02)
SHELF_POS = (BAG_POS[0], BAG_POS[1], SHELF_TOP - SHELF_SIZE[2] / 2)

BAG_MOUTH_Z = BAG_POS[2] + BAG_HALF_H
TABLET_DROP_H = 0.015
TABLET_POS = (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z + TABLET_DROP_H)

N_PREP = 200
N_DROP, N_SETTLE, N_CLOSE, N_GRASP, N_LIFT, N_HOLD = 150, 60, 80, 40, 200, 100
N_ABOVE, N_INSERT, N_SETTLE2 = 400, 400, 100
N_CLAMP, N_RELEASE = 400, 100

CAM_LOOK = tuple(FINGER_MID + np.array([0, 0, 0.03]))
OVERVIEW_CAM_POS = (0.9, -1.7, 1.3)
OVERVIEW_CAM_LOOK = (-0.15, -0.35, 0.15)
BAGCAM_OFFSET = np.array([0.20, -0.20, 0.12])


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def _prepare_robot_mjcf():
    src_dir = paths.ROBOTS_DIR
    tmp_dir = tempfile.mkdtemp(prefix="m0609_ipc_v2_")
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


def _prepare_seal_colored_bag():
    m = tm.load(BAG_STL)
    v = m.vertices
    u = np.clip((v[:, 0] + BAG_PANEL_HALF_W) / (2 * BAG_PANEL_HALF_W), 0, 1)
    vv = np.clip((v[:, 1] + BAG_PANEL_HALF_H) / (2 * BAG_PANEL_HALF_H), 0, 1)
    m.visual = tm.visual.TextureVisuals(uv=np.stack([u, vv], axis=1))
    obj_path = os.path.join(OUT_DIR, "_bag_seal_uv.obj")
    m.export(obj_path)

    tex_w = 128
    tex = np.tile(np.array(BAG_BODY_COLOR, dtype=np.uint8), (tex_w, tex_w, 1))
    u_axis = np.linspace(0, 1, tex_w)
    seal_frac = SEAL_BAND_WIDTH / (2 * BAG_PANEL_HALF_W)
    seal_cols = (u_axis < seal_frac) | (u_axis > 1 - seal_frac)
    tex[:, seal_cols] = np.array(SEAL_COLOR, dtype=np.uint8)
    return obj_path, tex


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" [IPC] Legacy_vs_IPC pipeline_ipc  (viewer={use_viewer})")
    print("=" * 60)

    t_wall0 = time.perf_counter()
    phase_times = {}

    crusher_xml = _prepare_crusher_mjcf()
    robot_xml = _prepare_robot_mjcf()
    bag_obj, bag_seal_tex = _prepare_seal_colored_bag()

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")
    patch_fem_vertex_constraints()

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=IPC_D_HAT,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=False,
            constraint_strength_translation=100.0,
            constraint_strength_rotation=100.0,
        ),
        fem_options=gs.options.FEMOptions(damping=FEM_DAMPING),
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
            material=gs.materials.Rigid(coup_type="ipc_only"),
            surface=gs.surfaces.Default(color=(0.82, 0.82, 0.85), metallic=0.85, roughness=0.3),
        )

    crusher = scene.add_entity(
        gs.morphs.MJCF(file=crusher_xml, pos=CRUSHER_POS, euler=CRUSHER_EULER,
                       decimate=True, convexify=True),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
        surface=gs.surfaces.Default(smooth=False),
    )

    scene.add_entity(gs.morphs.Plane(visualization=False), material=gs.materials.Rigid(coup_type="ipc_only"))
    scene.add_entity(
        gs.morphs.Box(size=SHELF_SIZE, pos=SHELF_POS, fixed=True),
        material=gs.materials.Rigid(coup_type="ipc_only", coup_friction=0.3),
        surface=gs.surfaces.Default(color=(0.75, 0.78, 0.82)),
    )

    robot = scene.add_entity(
        gs.morphs.MJCF(file=robot_xml, pos=tuple(ROBOT_OFFSET), decimate=False),
        material=gs.materials.Rigid(
            coup_type="two_way_soft_constraint",
            coup_links=FINGER_LINKS,
            coup_friction=CLOTH_FRICTION,
        ),
    )

    bag = scene.add_entity(
        material=gs.materials.FEM.Cloth(
            E=CLOTH_E, nu=CLOTH_NU, rho=CLOTH_RHO,
            thickness=CLOTH_THICK, bending_stiffness=CLOTH_BEND,
            friction_mu=CLOTH_FRICTION,
        ),
        morph=gs.morphs.Mesh(file=bag_obj, scale=BAG_SCALE, pos=BAG_POS, euler=BAG_EULER),
        surface=gs.surfaces.Default(opacity=0.55, roughness=0.9, double_sided=True,
                                     diffuse_texture=gs.textures.ImageTexture(image_array=bag_seal_tex)),
    )

    cap_verts_mm, cap_elems = make_capsule_tets_v2(
        radius_mm=CAP_RADIUS_MM, cyl_height_mm=CAP_CYL_H_MM, n_theta=12, n_cap_rings=4, n_cyl_bands=2,
    )
    tablet = add_analytic_fem_entity(
        scene, key=os.path.join(OUT_DIR, "_analytic_capsule_v2.stl"),
        verts_mm=cap_verts_mm, elems=cap_elems,
        material=gs.materials.FEM.Elastic(
            E=TABLET_E, nu=TABLET_NU, rho=TABLET_RHO,
            friction_mu=TABLET_FRICTION, model="stable_neohookean",
        ),
        scale=1e-3, pos=TABLET_POS,
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

    bag_pos0 = _npy(bag.get_state().pos).squeeze()
    by, bz = bag_pos0[:, 1], bag_pos0[:, 2]
    bag_bottom_mask = bz < bz.min() + 0.012
    bag_side_mask = (by < by.min() + 0.008) | (by > by.max() - 0.008)
    bag_fixed_idx = np.where(bag_bottom_mask | bag_side_mask)[0]
    bag.set_vertex_constraints(verts_idx_local=bag_fixed_idx.tolist(), is_soft_constraint=False)
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

    gripper_link = robot.get_link("gripper_body")
    left_link = robot.get_link(FINGER_LINKS[0])

    q_grasp, q_lift = Q_GRASP, Q_LIFT
    robot.set_dofs_position(np.concatenate([q_grasp, [FING_OPEN] * 6]))

    cam_over.start_recording()
    cam_bag.start_recording()

    def _bag_com():
        p = _npy(bag.get_state().pos).squeeze()
        return p.mean(axis=0)

    def _tablet_z():
        p = _npy(tablet.get_state().pos).squeeze()
        return float(p[:, 2].mean())

    def _finger_z():
        return float(_npy(left_link.get_pos()).squeeze()[2])

    def render_cams():
        cam_over.render()
        bc = _bag_com()
        cam_bag.set_pose(pos=tuple(bc + BAGCAM_OFFSET), lookat=tuple(bc), up=(0, 0, 1))
        cam_bag.render()

    def run_arm(name, q0, q1, f0, f1, n, crank_q=None, wall_q=None, trace=False):
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
            if trace and k % 40 == 0:
                print(f"    [{name} k={k:4d}] tablet_z={_tablet_z()*1e3:+.2f}mm bag_com={_bag_com()}")
        phase_times[name] = time.perf_counter() - t0
        bc = _bag_com()
        print(f"[phase] {name:8s} @done  bag_com={bc}  finger_z={_finger_z():.4f}  "
              f"tablet_z={_tablet_z()*1e3:+.2f}mm  ({phase_times[name]:.1f}s wall)")

    print(f"\n[phase] 0 prep ({N_PREP*DT:.1f}s sim) — 크랭크 0->{CRANK_START_Q:+.3f}rad, "
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
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)

    bag.remove_vertex_constraints()
    print("[bag] shape 고정 해제 — 이제부터 순수 마찰 파지")

    run_arm("close", q_grasp, q_grasp, FING_OPEN, FING_CLOSE, N_CLOSE,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)
    run_arm("grasp", q_grasp, q_grasp, FING_CLOSE, FING_CLOSE, N_GRASP,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)
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

    print(f"\n[phase] 9 clamp ({N_CLAMP*DT:.1f}s sim) — Left_Wall {WALL_OFFSET*1000:+.1f}mm -> "
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
            crank_q=CRANK_START_Q, wall_q=CLAMP_TARGET, trace=True)

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
        "combo": "ipc",
        "coupler": "IPCCoupler",
        "bag_material": "FEM.Cloth",
        "tablet_material": "FEM.Elastic",
        "grasp_method": "friction_contact (no cheat)",
        "dt": DT,
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

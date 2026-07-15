"""
scene_setup.py — Crusher_M0609_RG2_Tablet_Samplebag 통합 씬.

구성:
  - 알루미늄 plate 4개(2×2 그리드, Crusher_Pill.py 검증 패턴).
  - Crusher: 원점(0,0,0)에 배치(euler=(0,0,90), 기존 프로덕션 스크립트와 동일
    방향 관례). MJCF 패치는 Crusher_Pill.py::patch_crusher_mjcf 그대로.
  - M0609+RG2(v2, mimic joint + CoACD convex decomposition) + 정제(FEM.Elastic,
    sliver-free 캡슐) + 샘플백(FEM.Cloth): `M0609_RG2_Tablet_Samplebag/
    tablet_bag_grasp_pipeline.py`(§docs/DigitalTwin.md §9 조합8, 검증됨)를
    그대로 재사용하되, Crusher/plate 와 겹치지 않도록 로봇 베이스를
    `ROBOT_OFFSET`만큼 평행이동한다(FINGER_MID 등 로봇 기준 상대 배치는
    전부 FINGER_MID 로부터 파생되므로, FINGER_MID 자체를 오프셋하면 나머지는
    그대로 따라간다).

**미검증 조합 경고**: Crusher(순수 Rigid, weld equality 로 크랭크-슬라이더
폐루프를 풂, `coup_type` 미지정)를 IPC 커플러가 활성화된 씬(정제/봉투 FEM +
로봇 two_way_soft_constraint)에 **처음으로** 같이 넣는다. `coup_type`을
지정 안 한 Rigid 엔티티가 IPC 활성 씬에서 정확히 어떻게 취급되는지(IPC가
아예 무시하고 Genesis 네이티브 rigid 파이프라인에만 맡기는지, 아니면
빌드 자체가 실패하는지) 확인된 바 없다 — 이 스크립트의 1차 목적은 그
확인이다(§docs/DigitalTwin.md §9 조합8에서 "동적 ipc_only + two_way_soft_
constraint" 혼재만으로도 새 불안정성이 나왔던 전례가 있어 신중히 접근).

출력: RESULT/scene_setup_snapshot.png (빌드+배치 확인용, 아직 물리 스텝은
안 돌림 — 안정성 확인 전까지는 워크플로우 실행을 보류).
"""
import os, sys, shutil, tempfile
import xml.etree.ElementTree as ET
import numpy as np

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

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)

# ── Crusher + 알루미늄 plate (Crusher_Pill.py 검증 패턴) ────────────────────
CRUSHER_SRC_XML = paths.MJCF_MAIN
CRUSHER_POS = (0.0, 0.0, 0.0)
CRUSHER_EULER = (0.0, 0.0, 90.0)

PLATE_PATH = paths.ALUMINUM_PLATE
PLATE_POSITIONS = [(0.5, -0.5, 0), (0.5, 0.5, 0), (-0.5, -0.5, 0), (-0.5, 0.5, 0)]

WALL_GEOMS_TO_ENABLE = {"base_link", "L1_Wall1_1", "L1_Wall2_1", "L2_Wall3_1"}
L7_LINK3_COM = "0.006 0 -0.005"


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
    tmp_dir = tempfile.mkdtemp(prefix="crusher_mjcf_")
    src_dir = os.path.dirname(CRUSHER_SRC_XML)
    for f in os.listdir(src_dir):
        s = os.path.join(src_dir, f)
        if os.path.isfile(s):
            shutil.copy2(s, os.path.join(tmp_dir, f))
    dst = os.path.join(tmp_dir, "Crusher_genesis.xml")
    patch_crusher_mjcf(CRUSHER_SRC_XML, dst)
    return dst


# ── M0609+RG2(v2) + 정제 + 샘플백 (tablet_bag_grasp_pipeline.py 그대로,
# 로봇 베이스만 ROBOT_OFFSET 만큼 평행이동해 Crusher/plate 와 안 겹치게 함) ──
ROBOT_MJCF = os.path.join(paths.ROBOTS_DIR, "m0609_rg2_v2.xml")
COACD_DIR_REL = "rg2/reference_onrobot_ros/meshes/rg2_v1/coacd"
FLEX_FINGER_HULLS = [f"flex_finger_hull_{i:03d}.stl" for i in range(7)]
BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag", "Samplebag_seal_pouch3.stl")

CAP_RADIUS_MM, CAP_CYL_H_MM = 2.0, 1.0
TABLET_E, TABLET_NU, TABLET_RHO = 5.0e4, 0.45, 1300.0
TABLET_FRICTION = 0.5

DT = 5e-3
IPC_D_HAT = 1.0e-4

CLOTH_E, CLOTH_NU, CLOTH_RHO = 1.0e5, 0.499, 200.0
CLOTH_THICK, CLOTH_BEND = 1.0e-3, 50.0
CLOTH_FRICTION = 0.8

FINGER_LINKS = ("f1_flex_finger", "f2_flex_finger")
Q_GRASP = np.array([0, -0.40, 1.30, 0, 2.00, np.pi / 2], float)
Q_LIFT = np.array([0, -0.11, 0.60, 0, 2.41, np.pi / 2], float)
FING_OPEN, FING_CLOSE = 1.00, 1.20

# plate 1개가 1m×1m(실측: aluminum_plate.stl bounds ±0.5m) 라서 4개 그리드가
# 덮는 작업면은 X,Y ∈ [-1, 1] 전체다 — 이전 버전은 ROBOT_OFFSET=(1.6,0,0) 로
# 이 범위 밖(plate 밖 맨바닥)에 로봇을 놨었다("알루미늄 플레이트 밖에 로봇암이
# 설치되어있다" 지적, 2026-07-15) — Crusher(원점)와 안 겹치면서 plate 범위
# [-1,1] 안에 들어오도록 Y축으로 옆으로 옮긴다. 로봇 자체는 회전 없이
# 이동만 하므로 FINGER_MID 를 그대로 오프셋하면 BAG_POS/SHELF_POS/TABLET_POS
# 등 파생값이 전부 따라간다.
ROBOT_OFFSET = np.array([0.0, 0.7, 0.0])
FINGER_MID_BASE = np.array([0.20365, 0.00618, 0.43297])
FINGER_MID = FINGER_MID_BASE + ROBOT_OFFSET

BAG_SCALE = 1.0
BAG_EULER = (90, 0, 0)
SEAL_LOCAL_X = -0.028
BAG_POS = (FINGER_MID[0] - SEAL_LOCAL_X, FINGER_MID[1], FINGER_MID[2])
BAG_HALF_H = 0.045

SHELF_TOP = BAG_POS[2] - BAG_HALF_H - 0.0015
SHELF_SIZE = (0.10, 0.10, 0.02)
SHELF_POS = (BAG_POS[0], BAG_POS[1], SHELF_TOP - SHELF_SIZE[2] / 2)

BAG_MOUTH_Z = BAG_POS[2] + BAG_HALF_H
TABLET_DROP_H = 0.015
TABLET_POS = (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z + TABLET_DROP_H)

CAM_POS = tuple(np.array([0.75, -0.55, 0.70]) + ROBOT_OFFSET)
CAM_LOOK = tuple(FINGER_MID + np.array([0, 0, 0.03]))
# 전체 조망용 카메라(Crusher~로봇 워크스페이스 다 보이게, plate 전체[-1,1] 프레이밍)
OVERVIEW_CAM_POS = (-1.6, -2.2, 2.0)
OVERVIEW_CAM_LOOK = (0.0, ROBOT_OFFSET[1] / 2, 0.15)


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def _prepare_robot_mjcf():
    src_dir = paths.ROBOTS_DIR
    tmp_dir = tempfile.mkdtemp(prefix="m0609_cts_v2_")
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


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" Crusher_M0609_RG2_Tablet_Samplebag — full scene build check (viewer={use_viewer})")
    print("=" * 60)

    crusher_xml = _prepare_crusher_mjcf()
    robot_xml = _prepare_robot_mjcf()
    print(f"[crusher] patched MJCF -> {crusher_xml}")
    print(f"[robot]   patched MJCF -> {robot_xml}")
    print(f"[robot]   offset by {ROBOT_OFFSET.tolist()}  FINGER_MID={FINGER_MID.tolist()}")

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

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
        vis_options=gs.options.VisOptions(background_color=(0.93, 0.94, 0.96)),
        show_viewer=use_viewer,
    )

    # ── 알루미늄 plate 4개 ──────────────────────────────────────────────────
    # coup_type 미지정 시 IPC 자동선택이 이 조합(Crusher+로봇 two_way_soft_
    # constraint + FEM 둘 다 있는 씬)에서 CUDA 레벨 하드 크래시를 일으켰다
    # (cudaErrorInvalidDevice, scene.build() 내부 advance() 호출 중). 격리
    # 테스트로 원인을 plate 로 확정 — Plane/Shelf 와 동일하게 정적 소품이니
    # coup_type="ipc_only" 명시로 해결.
    for p in PLATE_POSITIONS:
        scene.add_entity(
            gs.morphs.Mesh(file=PLATE_PATH, fixed=True, pos=p),
            material=gs.materials.Rigid(coup_type="ipc_only"),
            surface=gs.surfaces.Default(color=(0.82, 0.82, 0.85), metallic=0.85, roughness=0.3),
        )
    print(f"[plates] {len(PLATE_POSITIONS)}개 배치(coup_type=ipc_only): {PLATE_POSITIONS}")

    # ── Crusher(원점) ────────────────────────────────────────────────────
    # coup_type 미지정 시 자동선택이 "fixed-base articulated" -> external_
    # articulation 을 골랐는데, 이건 모든 링크에 collision geometry 가 있어야
    # 함(Crusher 는 장식용/비충돌 링크가 많아 불만족) -> 빌드 자체가 실패했다
    # (GenesisException: Rigid link has no collision geometry). two_way_soft_
    # constraint 로 명시 override.
    crusher = scene.add_entity(
        gs.morphs.MJCF(file=crusher_xml, pos=CRUSHER_POS, euler=CRUSHER_EULER,
                       decimate=True, convexify=True),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
        surface=gs.surfaces.Default(smooth=False),
    )
    print(f"[crusher] pos={CRUSHER_POS}  euler={CRUSHER_EULER}  coup_type=two_way_soft_constraint")

    # ── 로봇 워크스페이스 바닥(shelf) + Plane(ipc_only) ─────────────────────
    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(coup_type="ipc_only"))
    scene.add_entity(
        gs.morphs.Box(size=SHELF_SIZE, pos=SHELF_POS, fixed=True),
        material=gs.materials.Rigid(coup_type="ipc_only", coup_friction=0.3),
        surface=gs.surfaces.Default(color=(0.75, 0.78, 0.82)),
    )

    # ── M0609+RG2(v2) ────────────────────────────────────────────────────
    robot = scene.add_entity(
        gs.morphs.MJCF(file=robot_xml, pos=tuple(ROBOT_OFFSET), decimate=False),
        material=gs.materials.Rigid(
            coup_type="two_way_soft_constraint",
            coup_links=FINGER_LINKS,
            coup_friction=CLOTH_FRICTION,
        ),
    )

    # ── 샘플백(FEM.Cloth) ────────────────────────────────────────────────
    bag = scene.add_entity(
        material=gs.materials.FEM.Cloth(
            E=CLOTH_E, nu=CLOTH_NU, rho=CLOTH_RHO,
            thickness=CLOTH_THICK, bending_stiffness=CLOTH_BEND,
            friction_mu=CLOTH_FRICTION,
        ),
        morph=gs.morphs.Mesh(file=BAG_STL, scale=BAG_SCALE, pos=BAG_POS, euler=BAG_EULER),
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.55,
                                     roughness=0.9, double_sided=True),
    )

    # ── 정제(FEM.Elastic, sliver-free 캡슐) ─────────────────────────────
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

    cam = scene.add_camera(res=(960, 720), pos=CAM_POS, lookat=CAM_LOOK, fov=45, GUI=False)
    cam_over = scene.add_camera(res=(1280, 960), pos=OVERVIEW_CAM_POS, lookat=OVERVIEW_CAM_LOOK,
                                fov=50, GUI=False)

    print("\n[build] scene.build() 시작...")
    scene.build(n_envs=0)
    print("[build] 성공 — Crusher + M0609/RG2 + 정제 + 샘플백 + plate 4개, 전부 빌드 OK")

    robot.set_dofs_position(np.concatenate([Q_GRASP, [FING_OPEN] * 6]))

    from PIL import Image
    for name, c in (("workspace", cam), ("overview", cam_over)):
        img = c.render()
        rgb = img[0] if isinstance(img, (tuple, list)) else img
        rgb = _npy(rgb)
        rgb = rgb[0] if rgb.ndim == 4 else rgb
        Image.fromarray(rgb[..., :3].astype("uint8")).save(
            os.path.join(OUT_DIR, f"scene_setup_{name}.png"))
        print(f"[saved] {os.path.join(OUT_DIR, f'scene_setup_{name}.png')}")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

"""
full_workflow.py — 정제 낙하 -> 봉투가 받음 -> M0609+RG2 가 봉투를 파지·리프트
-> Crusher 슬롯(Wall3~Left_Wall gap)까지 옮겨서 삽입.

Crusher_Samplebag.py(§PBD 버전, "박스+봉투를 carrier 로 슬롯에 삽입" 시도)를
참고해 슬롯 위치 계산·크랭크/슬라이더 프리셋 로직을 그대로 가져오고, 봉투
이동 수단만 carrier 텔레포트 대신 **M0609+RG2 로봇이 실제로 파지해서 옮기는
것**으로 바꿨다(§docs/DigitalTwin.md §9 조합8 파이프라인 재사용).

시퀀스:
  1. prep      : 슬롯에 넣기 전 준비 — 크랭크 0→-180°(CRANK_START_Q, L8 크러싱헤드
                 완전 후퇴), Left_Wall 0→+6mm(WALL_OFFSET, 슬롯 개방). 이 두 동작이
                 끝나야 슬롯이 "봉투가 통과할 수 있는" 상태가 된다(사용자 지시).
                 정제/봉투는 아직 로봇 워크스페이스에 그대로.
  2. drop/settle/close/grasp/lift/hold : tablet_bag_grasp_pipeline.py 그대로
                 (정제 낙하 -> 봉투 안착 -> 그리퍼 파지 -> 리프트).
  3. above     : 그리퍼(+봉투+정제)를 슬롯 바로 위(gap_cx, gap_cy, 0.30)로 이동.
                 목표 EE pose 는 역기구학(inverse_kinematics)으로 계산 — Q_LIFT
                 자세의 orientation 을 유지한 채 position 만 슬롯 위로 바꾼다.
  4. insert    : 슬롯 안(gap_cx, gap_cy, 0.15, 부분 삽입 — Crusher_Samplebag.py
                 와 동일하게 완전 삽입은 안 함)까지 하강.

**봉투 형상 고정(2026-07-15 추가)**: 1차 시도에서 봉투가 아무 지지 없이
중력만으로 버티다 정제가 들어가기도 전에 처져버려(§docs/DigitalTwin.md §9),
이후 grasp/lift 전부 실패했다(그리퍼는 126mm 올라갔는데 봉투는 그 자리에
그대로). 원인 진단 결과 **봉투를 고정하는 함수가 아예 없었다** — grasp는
처음부터 끝까지 순수 마찰 접촉이었는데(치트 없음), 그 전에 형상 자체가
무너져 있었던 것.
Genesis `FEMEntity.set_vertex_constraints()`가 정확히 이 용도(PBD의
`fix_particles_to_link` 대응)지만, **IPC 커플러 사용 시 예외를 던지는
버그**(`fem_entity.py` `isinstance(coupler, IPCCoupler)` 체크가 뒤집혀 있음,
실측 확인)로 우리 씬(IPC)에서는 그대로 못 쓴다. `utills/fem_ipc_workarounds.
patch_fem_vertex_constraints()`로 이 버그만 우회하는 몽키패치를 적용한다
(Genesis 설치본은 안 건드림).
바닥 밴드만 고정하면 위쪽(입구)이 결국 접혀 무너지길래(격리 테스트 확인),
**바닥 + 양 측면**(정점의 39% 정도, 입구 쪽은 완전히 자유)을 고정 — 설계
치수(90mm)를 1초 이상 정확히 유지함을 격리 테스트로 확인했다. `prep`~
`settle`까지 고정 유지, `close`(그리퍼 닫기) 직전에 `remove_vertex_
constraints()`로 전부 해제 — 이후 grasp/lift는 여전히 순수 마찰로 진행.

카메라 2대: overview(고정 광각, Crusher+로봇 전체) + bagcam(매 프레임 봉투
COM 을 추적하는 동적 카메라).

**후속 튜닝(2026-07-15 2차)**: 사용자 시각 검수 결과 5가지 수정.
  1. 봉투가 above/insert 구간에서 과하게 흔들림 -> CLOTH_E 1e5->4e5,
     CLOTH_BEND 50->400 로 stiffen + FEM_DAMPING=0.2 추가.
  2. 정제가 우그러진 형상으로 보임 -> IPC_D_HAT 1e-4(정제 극 근처 최소 정점
     간격 0.32mm 대비 비율 0.31, 경계값)을 5e-5(비율 0.16)로 낮춤(§docs/
     DigitalTwin.md §9 조합5 절차 재적용).
  3. 격자무늬 Ground 가 렌더에 보임 -> `gs.morphs.Plane(visualization=False)`
     로 충돌(안전망)은 유지하되 렌더만 끔 — 씬엔 알루미늄 플레이트만 보임.
  4. 조명이 어두움 -> ambient_light 상향 + 45도 방향 키 라이트(intensity 8)
     + 반대편 약한 필 라이트(intensity 3) 추가.
  5. 슬롯 삽입이 가장 불안정 -> N_ABOVE/N_INSERT 200->400(더 천천히) +
     above/insert 의 목표 orientation 을 Q_LIFT 그대로(wrist=+90°) 대신
     wrist=0 으로 되돌린 순간의 gripper quat 으로 교체(불필요한 90도 twist
     제거, 팔이 덜 웅크리게 됨).

**주의**: 로봇 원래 위치((0, 0.7, 0), scene_setup.py)는 슬롯(약 (-0.33,-0.05,
0.09))과의 거리가 0.87m로 이 자세(orientation 고정)에서 IK 오차가 12cm까지
나서, 로봇을 슬롯에 더 가까운 (-0.33, -0.65, 0) 로 재배치했다(오차 <0.001m
확인). scene_setup.py 의 배치와는 다르다.

출력: RESULT/full_workflow_<ts>_overview.mp4, RESULT/full_workflow_<ts>_bagcam.mp4
"""
import os, sys, shutil, tempfile
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
MP4_OVERVIEW = os.path.join(OUT_DIR, f"full_workflow_{_TS}_overview.mp4")
MP4_BAGCAM = os.path.join(OUT_DIR, f"full_workflow_{_TS}_bagcam.mp4")

# ── Crusher + plate (scene_setup.py 동일) ───────────────────────────────────
CRUSHER_SRC_XML = paths.MJCF_MAIN
CRUSHER_POS = (0.0, 0.0, 0.0)
CRUSHER_EULER = (0.0, 0.0, 90.0)
PLATE_PATH = paths.ALUMINUM_PLATE
PLATE_POSITIONS = [(0.5, -0.5, 0), (0.5, 0.5, 0), (-0.5, -0.5, 0), (-0.5, 0.5, 0)]

WALL_GEOMS_TO_ENABLE = {"base_link", "L1_Wall1_1", "L1_Wall2_1", "L2_Wall3_1"}
L7_LINK3_COM = "0.006 0 -0.005"

# ── Crusher 슬롯 계산에 필요한 벽 mesh(Crusher_Samplebag.py 동일) ──────────
WALL_BACK_MESH = "L2_Wall3_1"
WALL_LEFT_MESH = "L2_Left_Wall1_1"
LEFTWALL_BODY_POS = (-0.017802, 0.286278, 0.016542)
LEFTWALL_GEOM_POS = (-0.286278, -0.016542, 0.017802)
_R_GEOM_HALF = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])

CRANK_JOINT = "L3_Bevel_GearBox_1_L4_Shaft_1"
WALL_JOINT = "L1_Guide1_1_L2_Left_Wall1_1"
CRANK_START_Q = -np.pi   # -180 deg: L8 크러싱헤드 완전 후퇴(슬롯 밖)
WALL_OFFSET = 0.006      # 슬롯 개방(+6mm)
# Left_Wall(Motor2, Rack&Pinion) = 실제 봉투 고정 기구(docs/Crusher.md §5, §11-5:
# "샘플백을 단단히 고정...모터를 계속해서 구동시킴을 통해서 강하게 고정"). 로봇은
# 봉투를 gap 근처까지만 넣어주면 되고, 이후 이 벽이 닫히며 실링부를 Wall3 에
# 눌러 고정한다 — Crusher_Samplebag.py 의 CLAMP_TARGET 재사용(개방 +6mm →
# 클램프 -5mm, 총 11mm 이동해 6mm 두께 봉투를 압착).
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
    tmp_dir = tempfile.mkdtemp(prefix="crusher_mjcf_fw_")
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


# ── M0609+RG2(v2) + 정제 + 샘플백 ───────────────────────────────────────────
ROBOT_MJCF = os.path.join(paths.ROBOTS_DIR, "m0609_rg2_v2.xml")
COACD_DIR_REL = "rg2/reference_onrobot_ros/meshes/rg2_v1/coacd"
FLEX_FINGER_HULLS = [f"flex_finger_hull_{i:03d}.stl" for i in range(7)]
BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag", "Samplebag_seal_pouch3.stl")
# 실링부 색칠(2026-07-15 4차, 사용자 지시): Genesis 는 로드된 mesh 의
# vertex_colors 를 그대로 렌더에 반영하지 않는다(격리 테스트로 확인 — PLY에
# vertex_colors 를 구워도 단색으로만 나옴). UV + ImageTexture 조합만 동작
# 확인됨(§isolated test) — 봉투 로컬 좌표(폭 ±32mm, 높이 ±45mm, trimesh 로
# 로드시 이미 원점 중심)에 평면 UV(u=x/64mm+0.5, v=y/90mm+0.5)를 구워 OBJ 로
# 내보내고, 그 좌우 가장자리(seal, |local_x|>22mm 부근)에 해당하는 U 구간만
# 색이 다른 텍스처를 입힌다.
BAG_PANEL_HALF_W, BAG_PANEL_HALF_H = 0.032, 0.045
SEAL_BAND_WIDTH = 0.010                     # 가장자리에서 10mm(§docs "~1cm")
BAG_BODY_COLOR = (247, 247, 242)
SEAL_COLOR = (200, 60, 40)

CAP_RADIUS_MM, CAP_CYL_H_MM = 2.0, 1.0
TABLET_E, TABLET_NU, TABLET_RHO = 5.0e4, 0.45, 1300.0
TABLET_FRICTION = 0.5

DT = 5e-3
# contact_d_hat 은 씬 전체(플레이트4개+Crusher+로봇+봉투+정제)의 모든 접촉쌍에
# 적용되는 커플러 전역 설정이라, 5e-5 로 낮췄더니 build() 내부 warm-start 솔브가
# 30분+ 로 폭증(직전 1e-4 실행은 빌드+전체스텝+인코딩 합쳐 16분18초). 정제 극
# 근처 최소 정점 간격(0.32mm) 대비 1e-4 의 비율은 0.31 로 이미 문서 기준
# "1/3 이하" 안전 마진 안이므로, 성능 회귀를 감수할 근거가 부족해 원복한다.
IPC_D_HAT = 1.0e-4

# 봉투 stiffen(2026-07-15): above/insert 구간에서 봉투가 과하게 흔들려 E/굽힘
# 강성을 올림(1e5->4e5, bend 50->400) — 이동 속도를 늦추는 것(N_ABOVE/N_INSERT)과
# 함께 스윙을 줄이기 위한 조합 처방.
CLOTH_E, CLOTH_NU, CLOTH_RHO = 4.0e5, 0.499, 200.0
CLOTH_THICK, CLOTH_BEND = 1.0e-3, 400.0
CLOTH_FRICTION = 0.8
FEM_DAMPING = 0.2

FINGER_LINKS = ("f1_flex_finger", "f2_flex_finger")
Q_GRASP = np.array([0, -0.40, 1.30, 0, 2.00, np.pi / 2], float)
Q_LIFT = np.array([0, -0.11, 0.60, 0, 2.41, np.pi / 2], float)
FING_OPEN, FING_CLOSE = 1.00, 1.20

# 슬롯(약 (-0.33,-0.05,0.09))에서 0.87m 떨어진 원래 위치((0,0.7,0))는 orientation
# 고정 IK 오차가 12cm 까지 났다 — 슬롯에 훨씬 가까운 위치로 재배치(오차 <0.001m
# 확인, 이 스크립트 개발 중 격리 테스트로 검증).
ROBOT_OFFSET = np.array([-0.330, -0.65, 0.0])
FINGER_MID_BASE = np.array([0.20365, 0.00618, 0.43297])
FINGER_MID = FINGER_MID_BASE + ROBOT_OFFSET

BAG_SCALE = 1.0
BAG_EULER = (90, 0, 0)
SEAL_LOCAL_X = -0.028
BAG_HALF_H = 0.045
# 파지 위치를 봉투 거의 최상단(입구 쪽)으로 이동(2026-07-15 4차, 사용자 지시).
# 이전엔 FINGER_MID 가 봉투 중간 높이(local_y=0)에 오도록 BAG_POS_z=FINGER_MID_z
# 였는데, 입구에서 8mm 안쪽(맨 가장자리는 잡을 재료가 부족해 8mm 마진)에 오도록
# BAG_POS(봉투 중심)를 그만큼 아래로 내린다.
TOP_GRIP_MARGIN = 0.008
BAG_POS = (FINGER_MID[0] - SEAL_LOCAL_X, FINGER_MID[1],
           FINGER_MID[2] - BAG_HALF_H + TOP_GRIP_MARGIN)

SHELF_TOP = BAG_POS[2] - BAG_HALF_H - 0.0015
SHELF_SIZE = (0.10, 0.10, 0.02)
SHELF_POS = (BAG_POS[0], BAG_POS[1], SHELF_TOP - SHELF_SIZE[2] / 2)

BAG_MOUTH_Z = BAG_POS[2] + BAG_HALF_H
TABLET_DROP_H = 0.015
TABLET_POS = (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z + TABLET_DROP_H)

N_PREP = 200
N_DROP, N_SETTLE, N_CLOSE, N_GRASP, N_LIFT, N_HOLD = 150, 60, 80, 40, 200, 100
# above/insert 를 2배로 늘려 슬롯 접근을 더 천천히(사용자 지시) — 봉투 스윙도 완화.
N_ABOVE, N_INSERT, N_SETTLE2 = 400, 400, 100
# clamp: Left_Wall 이 실링부를 누르는 구간(개방 대비 훨씬 짧고 정밀한 이동이라
# Crusher_Samplebag.py N_CLAMP=2000(@dt=1e-3, 2.0s)와 동일 시간이 되도록 환산).
N_CLAMP, N_RELEASE = 400, 100

CAM_LOOK = tuple(FINGER_MID + np.array([0, 0, 0.03]))
OVERVIEW_CAM_POS = (0.9, -1.7, 1.3)
OVERVIEW_CAM_LOOK = (-0.15, -0.35, 0.15)
BAGCAM_OFFSET = np.array([0.20, -0.20, 0.12])


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def _prepare_robot_mjcf():
    src_dir = paths.ROBOTS_DIR
    tmp_dir = tempfile.mkdtemp(prefix="m0609_fw_v2_")
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
    """실링부(좌우 가장자리)만 다른 색으로 보이는 봉투 mesh 를 준비한다.
    평면 UV 를 구워 OBJ 로 내보내고, 그 UV 에 맞춘 스트라이프 텍스처 이미지를
    함께 반환한다 — (obj_path, texture_rgb_array)."""
    m = tm.load(BAG_STL)  # welded 모델(773 근처, 얼굴 공유) — Genesis 로드와 동일 위상
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
    print(f" Full workflow: tablet drop -> bag catch -> grasp -> lift -> Crusher slot insert (viewer={use_viewer})")
    print("=" * 60)

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
            # 1차 조정(ambient 0.35 + key 8.0)이 명암 대비가 거의 없이 밋밋했다는
            # 피드백 반영 — ambient 를 낮추고 key/fill 강도 비율을 벌려 그림자를
            # 다시 살린다(45도 키 라이트는 유지).
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

    # visualization=False: 충돌(안전망)은 유지하되 격자무늬 렌더는 끔 — 씬에는
    # 알루미늄 플레이트만 보이도록(사용자 지시).
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
        # color= 대신 실링부 스트라이프가 구워진 UV 텍스처를 사용(§_prepare_seal_colored_bag).
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
    scene.build(n_envs=0)
    print("[build] 성공")

    # ── 봉투 형상 고정: 바닥+양측면(입구는 자유) — §docstring 참고 ───────────
    bag_pos0 = _npy(bag.get_state().pos).squeeze()
    bx, bz = bag_pos0[:, 0], bag_pos0[:, 2]
    bag_bottom_mask = bz < bz.min() + 0.012
    bag_side_mask = (bx < bx.min() + 0.008) | (bx > bx.max() - 0.008)
    bag_fixed_idx = np.where(bag_bottom_mask | bag_side_mask)[0]
    bag.set_vertex_constraints(verts_idx_local=bag_fixed_idx.tolist(), is_soft_constraint=False)
    print(f"[bag] shape 고정: {len(bag_fixed_idx)}/{len(bz)} 정점(바닥+양측면), 입구는 자유")

    # ── Crusher 슬롯 위치 계산 ───────────────────────────────────────────────
    wb_lo, wb_hi = crusher_mesh_world_aabb(WALL_BACK_MESH)
    wl_lo, wl_hi = crusher_mesh_world_aabb(WALL_LEFT_MESH, LEFTWALL_BODY_POS, LEFTWALL_GEOM_POS)
    gap_lo_x, gap_hi_x = sorted([wb_hi[0], wl_lo[0]])
    gap_cx = (gap_lo_x + gap_hi_x) / 2.0
    gap_width = gap_hi_x - gap_lo_x
    y_lo = max(wb_lo[1], wl_lo[1]); y_hi = min(wb_hi[1], wl_hi[1])
    gap_cy = (y_lo + y_hi) / 2.0
    wall_top_z = max(wb_hi[2], wl_hi[2])
    # L1_Wall1_1 = 포켓 바닥 플레이트(Crusher_Samplebag.py 주석: "L1_Wall1_1 은
    # 바닥 플레이트"). gap 슬릿 중심(gap_cx,gap_cy)은 봉투가 "통과하는" 위치일
    # 뿐, 포켓 바닥의 실제 중심과는 다르다(실측: -38mm 가량 로봇 반대쪽으로
    # 치우쳐 있음) — 사용자 지시대로 봉투 하단 목표를 포켓 바닥 중심으로 잡는다.
    w1_lo, w1_hi = crusher_mesh_world_aabb("L1_Wall1_1")
    pocket_cx = (w1_lo[0] + w1_hi[0]) / 2.0
    pocket_cy = (w1_lo[1] + w1_hi[1]) / 2.0
    print(f"[slot] gap_cx={gap_cx:.4f} gap_cy={gap_cy:.4f} gap_width={gap_width*1000:.1f}mm wall_top_z={wall_top_z:.4f}")
    print(f"[slot] pocket(L1_Wall1_1) center=({pocket_cx:.4f},{pocket_cy:.4f})")

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

    vp0 = _npy(bag.get_state().pos).squeeze()
    d_to_mid = np.linalg.norm(vp0 - FINGER_MID, axis=1)
    grip_idx = np.where(d_to_mid < 0.020)[0].astype(int)
    print(f"[bag] grip_strip verts near FINGER_MID: {len(grip_idx)}")

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
        bc = _bag_com()
        print(f"[phase] {name:8s} @done  bag_com={bc}  finger_z={_finger_z():.4f}  tablet_z={_tablet_z()*1e3:+.2f}mm")

    # ── Phase 0: prep — 크랭크 -180도, Left_Wall 개방(슬롯 준비, 사용자 지시) ──
    print(f"\n[phase] 0 prep ({N_PREP*DT:.1f}s) — 크랭크 0->{CRANK_START_Q:+.3f}rad(-180deg), "
          f"Left_Wall 0->{WALL_OFFSET*1000:+.0f}mm(개방)")
    for k in range(N_PREP):
        s = (k + 1) / N_PREP
        crusher.control_dofs_position(np.array([CRANK_START_Q * s]), dofs_idx_local=[crank_dof])
        crusher.control_dofs_position(np.array([WALL_OFFSET * s]), dofs_idx_local=[wall_dof])
        robot.set_dofs_position(np.concatenate([q_grasp, [FING_OPEN] * 6]))
        scene.step()
        render_cams()
    cq = _npy(crusher.get_dofs_position())[crank_dof]
    wq = _npy(crusher.get_dofs_position())[wall_dof]
    print(f"[phase] prep     @done  crank={cq:+.3f}rad  wall={wq*1000:+.2f}mm")

    # ── Phase 1-6: 정제 낙하 -> 봉투 파지 -> 리프트 (tablet_bag_grasp_pipeline.py 동일) ──
    run_arm("drop", q_grasp, q_grasp, FING_OPEN, FING_OPEN, N_DROP,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET, trace=True)
    run_arm("settle", q_grasp, q_grasp, FING_OPEN, FING_OPEN, N_SETTLE,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)

    # 정제가 안전하게 들어간 뒤 형상 고정 해제 — 이후 grasp/lift 는 순수 마찰로 진행.
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

    # ── Phase 7: above — 슬롯 바로 위로 이동 (IK) ──────────────────────────
    # Q_LIFT 의 마지막 관절(wrist roll)=+pi/2 를 그대로 목표 orientation 에
    # 쓰면 슬롯 접근 시 불필요하게 90도 틀어진 자세가 되어 IK 가 팔을 억지로
    # 웅크리게 만든다(사용자 시각 확인: above/insert 프레임에서 팔이 바닥 쪽으로
    # 접힌 부자연스러운 자세). 손목 관절만 0으로 되돌린 순간의 gripper quat 을
    # 구해 그 값을 above/insert 목표 orientation 으로 쓴다 — 그리퍼는 이미 봉투를
    # 문 상태이므로 이 조회는 순수 forward-kinematics 이며, 조회 직후 실제 상태를
    # 즉시 복원해 물리에는 영향 없다.
    q_insert_ref = q_lift.copy()
    q_insert_ref[5] = 0.0
    robot.set_dofs_position(np.concatenate([q_insert_ref, [FING_CLOSE] * 6]))
    q_insert_quat = _npy(gripper_link.get_quat()).squeeze()
    robot.set_dofs_position(np.concatenate([q_lift, [FING_CLOSE] * 6]))  # 실제 상태 복원

    # ── 목표 위치 재계산(2026-07-15 5차, docs/Crusher.md §5·§11-5 참고) ───────
    # 로봇이 봉투를 포켓 깊숙이(L1_Wall1_1 바닥판 중앙)까지 옮기려 했던 4차
    # 시도는 실측상 목표에 도달하지 못했다(insert 내내 x가 목표로 수렴하지 않고
    # 오히려 멀어짐 — 좁은 gap 통과 후 옆으로 더 들어가야 하는 경로라 벽에 막힘).
    # docs/Crusher.md §5("Left_Wall...랙-피니언 구동으로...분쇄 간격을 조정"),
    # §11-5("샘플백 홀더...Motor2...Rack and Pinion...모터를 계속해서 구동시킴을
    # 통해서 강하게 고정")를 보면 애초에 봉투를 포켓 안까지 밀어넣는 건 로봇의
    # 역할이 아니다 — **로봇은 gap 근처까지만 옮기고, Left_Wall이 닫히며(Motor2)
    # 실링부를 Wall3 에 눌러 고정**하는 게 실제 기구 설계다. 목표를
    # Crusher_Samplebag.py 원안(target_x=gap_cx, target_z=wall_top_z)으로
    # 되돌린다 — 파지 위치가 이제 봉투 최상단(입구)이라 gripper_z ≈ 입구 높이가
    # 그대로 wall_top_z 목표와 대응된다.
    above_z = wall_top_z + 0.20   # Crusher_Samplebag.py SLOT_DZ_START=0.20 재사용
    insert_z = wall_top_z
    target_xy = np.array([gap_cx, gap_cy])

    target_above = np.array([target_xy[0], target_xy[1], above_z])
    qpos_above = _npy(robot.inverse_kinematics(
        link=gripper_link, pos=target_above, quat=q_insert_quat, dofs_idx_local=np.arange(6)))[:6]
    print(f"\n[ik] above-slot target={target_above}  arm_q={qpos_above}")
    run_arm("above", q_lift, qpos_above, FING_CLOSE, FING_CLOSE, N_ABOVE,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET, trace=True)

    # ── Phase 8: insert — 슬롯 입구까지 하강(gap 근방, 느리게) ───────────────
    target_insert = np.array([target_xy[0], target_xy[1], insert_z])
    qpos_insert = _npy(robot.inverse_kinematics(
        link=gripper_link, pos=target_insert, quat=q_insert_quat, dofs_idx_local=np.arange(6)))[:6]
    print(f"[ik] insert target={target_insert}  arm_q={qpos_insert}")
    run_arm("insert", qpos_above, qpos_insert, FING_CLOSE, FING_CLOSE, N_INSERT,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET, trace=True)
    run_arm("settle2", qpos_insert, qpos_insert, FING_CLOSE, FING_CLOSE, N_SETTLE2,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)

    # ── Phase 9: clamp — Left_Wall(Motor2, Rack&Pinion) 닫아 실링부 고정 ─────
    # docs/Crusher.md §11-5: 래칫/락 없이 모터를 계속 구동해 강하게 고정하는
    # 방식 — Crusher_Samplebag.py 의 CLAMP_TARGET(-5mm) 재사용, WALL_OFFSET
    # (+6mm)에서 총 11mm 이동해 6mm 두께 봉투를 압착.
    print(f"\n[phase] 9 clamp ({N_CLAMP*DT:.1f}s) — Left_Wall {WALL_OFFSET*1000:+.1f}mm -> "
          f"{CLAMP_TARGET*1000:+.1f}mm (실링부 압착)")
    for k in range(N_CLAMP):
        s = (k + 1) / N_CLAMP
        wq = WALL_OFFSET + (CLAMP_TARGET - WALL_OFFSET) * s
        crusher.control_dofs_position(np.array([wq]), dofs_idx_local=[wall_dof])
        crusher.control_dofs_position(np.array([CRANK_START_Q]), dofs_idx_local=[crank_dof])
        robot.set_dofs_position(np.concatenate([qpos_insert, [FING_CLOSE] * 6]))
        scene.step()
        render_cams()
    wq_final = _npy(crusher.get_dofs_position())[wall_dof]
    print(f"[phase] clamp    @done  wall={wq_final*1000:+.2f}mm  bag_com={_bag_com()}")

    # ── Phase 10: release — 그리퍼 개방, 고정은 이제 Left_Wall 이 담당 ───────
    run_arm("release", qpos_insert, qpos_insert, FING_CLOSE, FING_OPEN, N_RELEASE,
            crank_q=CRANK_START_Q, wall_q=CLAMP_TARGET, trace=True)

    cam_over.stop_recording(save_to_filename=MP4_OVERVIEW, fps=30)
    cam_bag.stop_recording(save_to_filename=MP4_BAGCAM, fps=30)
    print(f"\n[saved] overview -> {MP4_OVERVIEW}")
    print(f"[saved] bagcam   -> {MP4_BAGCAM}")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

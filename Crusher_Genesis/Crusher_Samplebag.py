"""
Crusher_Samplebag.py — 샘플백을 Crusher 의 Wall_1↔Left_Wall gap 에 삽입하는 공정.

Crushing.py 를 기반으로 하되, 목표를 바꾼 변형:
  · 기존: 봉투를 L2_Wall3_1 "앞면"에 기대 놓음
  · 변경: 봉투를 L2_Wall3_1(back wall) 과 L2_Left_Wall1_1(Left Wall) **사이 gap**(≈12mm)
          중앙에 삽입. (분쇄 챔버 = 두 벽 사이 공간)

구성:
  · 알루미늄 플레이트 4장 (2×2, 각 1m×1m×2cm, z=0 상단 고정)
  · M0609 6-DOF 매니퓰레이터 + OnRobot RG2 그리퍼  (robots/m0609_rg2.xml)
  · PBD 샘플백 (5면체 cloth, 상단 개방)  + 내부 Rigid 박스   ← Crushing.py 대비 스케일 ↓
  · Crusher_IsaacSim (Wall_1 / Left_Wall / 크랭크슬라이더) ─ 로봇 정면

시퀀스 (Crushing.py 동일):
  dropin → close → csettle → grasp → lift → move → approach → settle → descend → release → hold
  (j2+j3+j5=2.90 고정 → EE orientation 보존)

핵심 처리:
  · 샘플백/박스 스케일 다운 (W,H ↓, D=6mm 는 PBD 하한이라 유지)
  · 삽입 타깃 = 두 벽 AABB 로 계산한 gap 중앙. 핑거는 gap(12mm)에 못 들어가므로
    벽 상단 위에 두고 봉투만 gap 으로 늘어뜨린다(Crushing.py 문제1 동일).
  · Crusher MJCF 런타임 패치 (lock_crank 제거, weld 강화, ground 제거, 벽 충돌 ON)

출력:
  Sim_result/Crusher_Samplebag_<ts>.mp4 + 페이즈별 키프레임 PNG
"""
import os, sys, shutil, tempfile
import xml.etree.ElementTree as ET
from datetime import datetime
import numpy as np
import trimesh as tm
from PIL import Image

# Windows 콘솔(cp949) 에서 한글/em-dash print 깨짐 방지 → stdout/stderr UTF-8 고정
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

# ── 시뮬 옵션 ────────────────────────────────────────────────────────────────
DT, SUBSTEPS = 1e-3, 10
RENDER_EVERY = 15

# ── 샘플백 (cloth 패널) — Crushing.py(50×80×6) 대비 스케일 ↓ ────────────────
#   D(두께)는 PBD 하한 주의: 앞/뒤 패널 간격이 2*particle_size(≈5.7mm) 보다 작으면
#   파티클-파티클 충돌제약이 즉시 위반(2.1 폭발) → 6mm 가 사실상 최소이므로 유지.
#   W,H 만 축소해 gap(≈12mm x / 65mm y) 에 깔끔히 들어가도록.
W, H, D = 0.03, 0.05, 0.006
NW, NH, ND = 6, 9, 2
PARTICLE_SIZE = 2.83e-3

# ── 로봇 waypoint (S=j2+j3+j5=2.90 → EE 자세 유지) ────────────────────────
# 초기 fallback. main() 에서 grid scan 으로 BAG_POS/gap 위치에 맞춰 재계산.
Q_GRASP_INIT = np.array([0, -0.40, 1.30, 0, 2.00, 0], float)
Q_LIFT_INIT  = np.array([0, -0.11, 0.60, 0, 2.41, 0], float)
Q_MOVE_INIT  = np.array([0, -0.05, 0.85, 0, 2.10, 0], float)
FING_OPEN, FING_CLOSE = 0.04, 0.006

# ── 샘플백/박스 spawn ────────────────────────────────────────────────────────
GRASP_XY    = np.array([0.30, 0.0]); BAG_MOUTH_Z = 0.20
BAG_POS     = (GRASP_XY[0], GRASP_XY[1], BAG_MOUTH_Z - H/2)
# 파지 대상: 봉투 "가로 중앙 + 상단" 좁은 strip
GRIP_X_WIDTH    = 0.005
GRIP_Z_FROM_TOP = 0.005
GRIP_LINK       = "rg2_left"
BOX_SIZE    = (0.015, 0.004, 0.015)     # bag scale 따라 box 도 축소 (was 0.025×0.004×0.025)
BOX_RHO     = 300.0
BOX_SPAWN   = (GRASP_XY[0], GRASP_XY[1], BAG_MOUTH_Z + 0.05)

# ── 페이즈 step ──────────────────────────────────────────────────────────────
N_DROP, N_CLOSE, N_CSET, N_GRASP, N_LIFT, N_MOVE, N_APPROACH, N_SETTLE, N_DESCEND, N_REL, N_HOLD = \
    500, 250, 150, 75, 450, 600, 350, 500, 350, 150, 150
# N_SETTLE: APPROACH 직후 공중에서 봉투 진동 안정화 (0.5 s)

# ── 카메라 ──────────────────────────────────────────────────────────────────
CAM_WIDE_POS  = np.array([1.15, -1.05, 1.05])
CAM_WIDE_LOOK = np.array([0.27, 0.0, 0.45])
CAM_TRACK_OFF = np.array([0.45, -0.42, 0.18])

# ── Crusher 배치 (바닥 위, 매니퓰레이터 정면) ──────────────────────────────
CRUSHER_POS   = (0.55, 0.0, 0.0)
CRUSHER_EULER = (0.0, 0.0, 90.0)

# ── PBD 봉투 강성 (Samplebag 에서 검증: 박스 안전 + 유연성 ↑) ──────────────
STRETCH_COMPLIANCE = 1e-3
BENDING_COMPLIANCE = 1e-3

# ── 삽입 타깃: Wall_1(L2_Wall3_1) ↔ Left_Wall(L2_Left_Wall1_1) gap 중앙 ─────
#   두 벽 모두 정적(Left_Wall 은 slide body 지만 시작 qpos=0 → 초기 자세 = 정지).
#   gap 은 ≈12mm(x) 라 핑거가 못 들어감 → 핑거 TCP 는 벽 상단(max-z) 위로 띄우고
#   봉투만 gap 으로 늘어뜨린다.
WALL_BACK_MESH    = "L2_Wall3_1"          # 정적 worldbody geom (body/geom 오프셋 0)
WALL_LEFT_MESH    = "L2_Left_Wall1_1"     # slide body — body/geom 오프셋 있음
# L2_Left_Wall1_1 의 MJCF body / geom 로컬 오프셋 (XML 에서 추출, qpos=0 기준)
LEFTWALL_BODY_POS = (-0.017802, 0.286278, 0.016542)
LEFTWALL_GEOM_POS = (-0.286278, -0.016542, 0.017802)
EE_LINK_NAME      = "rg2_left"            # 봉투는 핑거끝(rg2_left)에 weld → 핑거를 타깃
# gap 중앙 기준 핑거 TCP 목표.
#   xy: gap 중앙 (필요 시 미세 보정)
#   z : 벽 상단(max-z) 기준 위로 띄운다 → 핑거가 벽면과 충돌 안 함.
# 봉투는 핑거 링크 원점에서 수평으로 일정 오프셋(+62mm x, -13mm y)만큼 치우쳐 매달림
# (1차 실행 측정: bag_com - EE = [0.062,-0.013,0.001]). gap 에 봉투를 넣으려면 핑거
# 타깃을 그 반대로 이동 → 봉투(=핑거+오프셋)가 gap 중앙에 오게 한다.
SLOT_DX           = -0.062
SLOT_DY           =  0.013
SLOT_DZ_FINAL     =  0.02                 # 핑거를 벽 상단 위 20mm (봉투는 그 아래 gap 으로)
# 2-step IK: step1 = gap 바로 위(APPROACH), step2 = z 만 내려 DESCEND
APPROACH_DZ       =  0.12
# place 에서 EE(공구축 roll, j6)를 90° 돌려 봉투를 90° 회전 → 봉투 얇은 면(D=6mm)을
# gap 의 얇은 x-방향(≈12mm)에 정렬.
J6_PLACE          =  np.pi / 2.0          # +90° (정렬 안 맞으면 -np.pi/2)

# ── 경로 ────────────────────────────────────────────────────────────────────
_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
STL_PATH = os.path.join(OUT_DIR, "crusher_samplebag_open.stl")
_TS      = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4_PATH = os.path.join(OUT_DIR, f"Crusher_Samplebag_{_TS}.mp4")
PLATE_PATH = os.path.join(_DIR, "robots/assets/aluminum_plate.stl")
ROBOT_MJCF = "robots/m0609_rg2.xml"
CRUSHER_SRC_XML = os.path.join(_DIR, "MJCF", "Crusher_IsaacSim_colored.xml")


# ─────────────────────────────── helpers ────────────────────────────────────
def _panel(fn, nu, nv):
    t = []
    for i in range(nu):
        for j in range(nv):
            a, b = fn(i/nu, j/nv), fn((i+1)/nu, j/nv)
            c, d = fn((i+1)/nu, (j+1)/nv), fn(i/nu, (j+1)/nv)
            t += [[a, b, c], [a, c, d]]
    return t


def make_bag():
    tris = []
    tris += _panel(lambda u, v: np.array([u*W, v*H, 0.0]), NW, NH)
    tris += _panel(lambda u, v: np.array([u*W, v*H, D]),   NW, NH)
    tris += _panel(lambda u, v: np.array([u*W, 0.0, v*D]), NW, ND)
    tris += _panel(lambda u, v: np.array([0.0, u*H, v*D]), NH, ND)
    tris += _panel(lambda u, v: np.array([W,   u*H, v*D]), NH, ND)
    v = np.array([p for t in tris for p in t])
    f = np.arange(len(v)).reshape(-1, 3)
    m = tm.Trimesh(vertices=v, faces=f, process=False)
    m.merge_vertices(digits_vertex=7)
    m.vertices -= m.bounding_box.centroid
    m.export(STL_PATH)


# base_link: 원본은 collision-free(contype=0) → 바닥/구조 충돌 위해 활성화.
WALL_GEOMS_TO_ENABLE = {"base_link", "L1_Wall1_1", "L1_Wall2_1", "L2_Wall3_1"}
L7_LINK3_COM = "0.006 0 -0.005"


def patch_crusher_mjcf(src, dst,
                        eq_solref="0.0002 50",
                        eq_solimp="0.999 0.99999 1e-5"):
    """원본 Crusher MJCF → Genesis 호환 사본.

    - <equality><joint> (lock_crank polycoef): 제거
    - <equality><weld> 유지, solref/solimp 빡빡한 값으로 교체
    - <geom name="ground">: 알루미늄 플레이트와 중복이라 제거
    - 벽 geom (L1_Wall1_1, L1_Wall2_1, L2_Wall3_1) 의 충돌 활성화
    - L7_Link3_1 의 dubious CoM 을 bbox 중심으로 교체
    """
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


# RG2 핑거 충돌 — 기본 OFF(visual-only).
#   Twin.md §4: 얇은 이중벽 봉투(앞/뒤 6mm)를 핑거로 마찰 압착하면 2.1 압착 불안정
#   → PBD 폭발. placement 공정은 핑거를 visual-only 로 두고 weld(attach, fix_particles_
#   to_link)로 봉투를 운반한다(weld 는 충돌과 무관하게 링크에 고정). 격리 검증 결과
#   봉투 자체는 안정이며, 풀 씬 폭발 원인이 핑거 충돌이라 기본값을 OFF 로 둔다.
#   마찰 파지를 실험하려면 True (단 dt↓·substep↑·solver iter↑ 필요, Twin.md 2.1).
ENABLE_RG2_FINGER_COLLISION = False
RG2_GEOMS_TO_ENABLE = {"rg2_finger", "rg2_hand"}


def patch_robot_mjcf(src, dst):
    """m0609_rg2.xml 사본. ENABLE_RG2_FINGER_COLLISION 시 핑거/손 collision 활성화.

    원본 m0609_rg2.xml 은 핑거 geom 이 contype="0"(visual-only). True 일 때만
    contype/conaffinity 제거 → default 0xFFFF (충돌 ON).
    """
    tree = ET.parse(src); root = tree.getroot()
    if ENABLE_RG2_FINGER_COLLISION:
        wb = root.find("worldbody")
        if wb is not None:
            for g in wb.iter("geom"):
                if g.get("mesh") in RG2_GEOMS_TO_ENABLE:
                    g.attrib.pop("contype", None)
                    g.attrib.pop("conaffinity", None)
    tree.write(dst)


def _prepare_robot_mjcf():
    """robots/m0609_rg2.xml 디렉터리 복사 + 패치본 생성."""
    src_xml = os.path.join(_DIR, ROBOT_MJCF)
    src_dir = os.path.dirname(src_xml)
    tmp_dir = tempfile.mkdtemp(prefix="m0609_mjcf_")
    for root_dir, _, files in os.walk(src_dir):
        rel = os.path.relpath(root_dir, src_dir)
        dst_dir = os.path.join(tmp_dir, rel) if rel != "." else tmp_dir
        os.makedirs(dst_dir, exist_ok=True)
        for f in files:
            shutil.copy2(os.path.join(root_dir, f), os.path.join(dst_dir, f))
    dst = os.path.join(tmp_dir, "m0609_rg2_patched.xml")
    patch_robot_mjcf(src_xml, dst)
    return dst


def _npy(x): return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)
def _pos_of(b):
    x = _npy(b.get_particles_pos())
    return x[0] if x.ndim == 3 else x
def _lerp(a, b, s): return a + (b - a) * s
def _rgb_of(r):
    a = r[0] if isinstance(r, (tuple, list)) else r
    a = _npy(a)
    return a[..., :3].astype("uint8")


# worldbody 정적 벽 geom 공통 변환: quat="0.5 0.5 0.5 0.5" → R 아래, scale 1e-3
_R_GEOM_HALF = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])  # quat(.5,.5,.5,.5)


def crusher_mesh_world_aabb(mesh_name, body_pos=(0., 0., 0.), geom_pos=(0., 0., 0.)):
    """Crusher 정적 mesh geom 의 월드 AABB (해석적, build 불필요).

    Genesis 가 정적 충돌 geom 을 link 당 1개로 병합해 개별 mesh live 조회가 안 되므로,
    MJCF 변환(body quat .5.5.5.5, geom quat .5.5.5.5, scale 1e-3) + 엔티티 변환
    (CRUSHER_POS, yaw) 으로 STL 정점을 직접 월드로 보내 min/max 를 구한다.

    - L2_Wall3_1: body_pos=geom_pos=0 (worldbody 직속 geom).
    - L2_Left_Wall1_1: body/geom 오프셋 지정 (slide body, qpos=0 → 초기 자세).
    """
    yaw = np.radians(CRUSHER_EULER[2])
    R_e = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                    [np.sin(yaw),  np.cos(yaw), 0.],
                    [0., 0., 1.]])
    v = tm.load(os.path.join(_DIR, "MJCF", f"{mesh_name}.stl")).vertices * 0.001
    local = np.asarray(geom_pos) + v                       # geom 로컬 오프셋
    in_crusher = np.asarray(body_pos) + (_R_GEOM_HALF @ local.T).T  # body quat .5.5.5.5
    w = np.array(CRUSHER_POS) + (R_e @ in_crusher.T).T     # 엔티티 변환
    return w.min(axis=0), w.max(axis=0)


def main(use_viewer: bool = True):
    print("="*60); print(f" Crusher_Samplebag — bag → Wall_1↔Left_Wall gap (viewer={use_viewer})"); print("="*60)
    make_bag()
    crusher_xml = _prepare_crusher_mjcf()
    print(f"[crusher] patched MJCF → {crusher_xml}")
    robot_xml = _prepare_robot_mjcf()
    print(f"[robot]   patched MJCF → {robot_xml}  "
          f"(RG2 핑거 충돌 {'ON' if ENABLE_RG2_FINGER_COLLISION else 'OFF(visual-only)'})")

    import genesis as gs
    gs.init(backend=gs.cuda, logging_level="warning")

    scene_kwargs = dict(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, -9.81)),
        pbd_options=gs.options.PBDOptions(max_density_solver_iterations=2, particle_size=PARTICLE_SIZE),
        coupler_options=gs.options.LegacyCouplerOptions(rigid_pbd=True),
        vis_options=gs.options.VisOptions(background_color=(0.93, 0.94, 0.96)),
        show_viewer=use_viewer,
    )
    if use_viewer:
        scene_kwargs["viewer_options"] = gs.options.ViewerOptions(
            camera_pos=tuple(CAM_WIDE_POS), camera_lookat=tuple(CAM_WIDE_LOOK),
            camera_fov=46, max_FPS=60)
    scene = gs.Scene(**scene_kwargs)

    # 알루미늄 플레이트 — 2×2 그리드 (각 1m×1m)
    plate_positions = [
        ( 0.5, -0.5, 0.0), ( 0.5,  0.5, 0.0),
        (-0.5, -0.5, 0.0), (-0.5,  0.5, 0.0),
    ]
    for p in plate_positions:
        scene.add_entity(
            gs.morphs.Mesh(file=PLATE_PATH, fixed=True, pos=p),
            material=gs.materials.Rigid(),
            surface=gs.surfaces.Default(color=(0.82, 0.82, 0.85), metallic=0.85, roughness=0.3),
        )
    # M0609 + RG2 (핑거 충돌 활성화 패치본)
    robot = scene.add_entity(gs.morphs.MJCF(file=robot_xml, decimate=False))
    # Crusher (정면, faceted 정밀 렌더)
    crusher = scene.add_entity(
        gs.morphs.MJCF(file=crusher_xml, pos=CRUSHER_POS, euler=CRUSHER_EULER,
                       decimate=False, convexify=False),
        surface=gs.surfaces.Default(smooth=False),
    )
    # PBD 샘플백 — Samplebag 검증된 유연 강성 (스케일 ↓)
    bag = scene.add_entity(
        material=gs.materials.PBD.Cloth(
            stretch_compliance=STRETCH_COMPLIANCE,
            bending_compliance=BENDING_COMPLIANCE,
        ),
        morph=gs.morphs.Mesh(file=STL_PATH, scale=1.0, pos=BAG_POS, euler=(90, 0, 0)),
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.7, roughness=0.9, double_sided=True),
    )
    # 내용물 박스 (스케일 ↓)
    scene.add_entity(
        material=gs.materials.Rigid(rho=BOX_RHO),
        morph=gs.morphs.Box(size=BOX_SIZE, pos=BOX_SPAWN, fixed=False),
        surface=gs.surfaces.Default(color=(0.85, 0.35, 0.25)),
    )
    cam = scene.add_camera(res=(960, 720), pos=tuple(CAM_WIDE_POS),
                           lookat=tuple(CAM_WIDE_LOOK), fov=46, GUI=False)
    scene.build(n_envs=0)

    grip_link_idx = robot.get_link(GRIP_LINK).idx
    robot.set_dofs_position(np.concatenate([Q_GRASP_INIT, [FING_OPEN, FING_OPEN]]))

    # ── Wall_1(back) ↔ Left_Wall gap 위치 ───────────────────────────────────
    #   두 벽 모두 정적 → MJCF geom 변환으로 월드 AABB 해석 계산, gap 중앙을 타깃.
    ee_link = robot.get_link(EE_LINK_NAME)
    wb_lo, wb_hi = crusher_mesh_world_aabb(WALL_BACK_MESH)
    wl_lo, wl_hi = crusher_mesh_world_aabb(WALL_LEFT_MESH, LEFTWALL_BODY_POS, LEFTWALL_GEOM_POS)
    # gap = back wall 의 +x 면(wb_hi[0]) 과 left wall 의 -x 면(wl_lo[0]) 사이
    gap_lo_x, gap_hi_x = sorted([wb_hi[0], wl_lo[0]])
    gap_cx = (gap_lo_x + gap_hi_x) / 2.0
    # y 는 두 벽의 겹치는 구간 중앙
    y_lo = max(wb_lo[1], wl_lo[1]); y_hi = min(wb_hi[1], wl_hi[1])
    gap_cy = (y_lo + y_hi) / 2.0
    wall_top_z = max(wb_hi[2], wl_hi[2])
    slot_xy   = np.array([gap_cx + SLOT_DX, gap_cy + SLOT_DY])
    ee_final    = np.array([slot_xy[0], slot_xy[1], wall_top_z + SLOT_DZ_FINAL])
    ee_approach = ee_final + np.array([0.0, 0.0, APPROACH_DZ])
    gap_center  = np.array([gap_cx, gap_cy, (wb_lo[2] + wall_top_z) / 2.0])
    print(f"\n[wall] back '{WALL_BACK_MESH}'  AABB lo={np.round(wb_lo,4)} hi={np.round(wb_hi,4)}")
    print(f"[wall] left '{WALL_LEFT_MESH}'  AABB lo={np.round(wl_lo,4)} hi={np.round(wl_hi,4)}")
    print(f"[gap]  x=[{gap_lo_x:.4f},{gap_hi_x:.4f}] (w={1000*(gap_hi_x-gap_lo_x):.1f}mm)  "
          f"y=[{y_lo:.4f},{y_hi:.4f}]  center={np.round(gap_center,4)}")
    print(f"[gap]  wall_top_z={wall_top_z:.4f}  ee_final={np.round(ee_final,4)}  "
          f"ee_approach={np.round(ee_approach,4)}")

    # Genesis IK 가 이 robot/target 조합에서 비정상 — manual brute search 로 대체.
    # j2+j3+j5=2.90 invariant 유지 (EE 수직 자세 보존), j4=0 고정.
    def _find_q_place(target, j6=0.0):
        best_q, best_err, best_ee = None, float("inf"), None
        for j1 in np.linspace(-1.0, 1.0, 21):
            for j2 in np.linspace(-1.5, 0.6, 22):
                for j3 in np.linspace(0.1, 2.5, 25):
                    j5 = 2.90 - j2 - j3
                    if not (-1.5 < j5 < 3.5):
                        continue
                    q = np.array([j1, j2, j3, 0.0, j5, j6])
                    robot.set_dofs_position(np.concatenate([q, [FING_CLOSE, FING_CLOSE]]))
                    ee = _npy(ee_link.get_pos())
                    e = float(np.linalg.norm(ee - target))
                    if e < best_err:
                        best_err, best_q, best_ee = e, q.copy(), ee
        if best_q is not None:
            base = best_q
            for j1 in np.linspace(base[0]-0.1, base[0]+0.1, 11):
                for j2 in np.linspace(base[1]-0.1, base[1]+0.1, 11):
                    for j3 in np.linspace(base[2]-0.1, base[2]+0.1, 11):
                        j5 = 2.90 - j2 - j3
                        if not (-1.5 < j5 < 3.5):
                            continue
                        q = np.array([j1, j2, j3, 0.0, j5, j6])
                        robot.set_dofs_position(np.concatenate([q, [FING_CLOSE, FING_CLOSE]]))
                        ee = _npy(ee_link.get_pos())
                        e = float(np.linalg.norm(ee - target))
                        if e < best_err:
                            best_err, best_q, best_ee = e, q.copy(), ee
        return best_q, best_err, best_ee

    # ── 모든 waypoint Q 를 grid scan 으로 동적 계산 ──────────────────────────
    target_grasp = np.array([GRASP_XY[0], GRASP_XY[1], BAG_MOUTH_Z + 0.02])
    Q_GRASP, err_g, ee_g = _find_q_place(target_grasp, j6=0.0)
    target_lift  = target_grasp + np.array([0.0, 0.0, 0.20])
    Q_LIFT,  err_l, ee_l = _find_q_place(target_lift, j6=0.0)
    target_move  = (target_lift + ee_approach) / 2.0
    Q_MOVE,  err_m, ee_m = _find_q_place(target_move, j6=J6_PLACE/2.0)
    Q_APPROACH, err_a, ee_a = _find_q_place(ee_approach, j6=J6_PLACE)
    Q_DESCEND,  err_d, ee_d = _find_q_place(ee_final, j6=J6_PLACE)
    print(f"\n[waypoint scan]")
    print(f"  GRASP   q={np.round(Q_GRASP,3)} EE={np.round(ee_g,4)} (target {np.round(target_grasp,4)} err {err_g*1000:.1f}mm)")
    print(f"  LIFT    q={np.round(Q_LIFT,3)} EE={np.round(ee_l,4)} (target {np.round(target_lift,4)} err {err_l*1000:.1f}mm)")
    print(f"  MOVE    q={np.round(Q_MOVE,3)} EE={np.round(ee_m,4)} (target {np.round(target_move,4)} err {err_m*1000:.1f}mm)")
    print(f"  APPROACH q={np.round(Q_APPROACH,3)} EE={np.round(ee_a,4)} (target {np.round(ee_approach,4)} err {err_a*1000:.1f}mm)")
    print(f"  DESCEND  q={np.round(Q_DESCEND,3)} EE={np.round(ee_d,4)} (target {np.round(ee_final,4)} err {err_d*1000:.1f}mm)")

    robot.set_dofs_position(np.concatenate([Q_GRASP, [FING_OPEN, FING_OPEN]]))

    pos0 = _pos_of(bag); z, x, y = pos0[:, 2], pos0[:, 0], pos0[:, 1]
    band = np.where(z >= np.quantile(z, 0.92))[0]
    corners = list({int(i) for i in [
        band[np.argmin(x[band] + y[band])], band[np.argmax(x[band] - y[band])],
        band[np.argmax(y[band] - x[band])], band[np.argmax(x[band] + y[band])]]})
    print(f"[bag] N={pos0.shape[0]}  bbox X={x.min():.3f}~{x.max():.3f}  "
          f"Y={y.min():.3f}~{y.max():.3f}  Z={z.min():.3f}~{z.max():.3f}")
    print(f"[bag] corners={len(corners)} positions:")
    for c in corners:
        print(f"        idx={c}  pos=({pos0[c,0]:.4f},{pos0[c,1]:.4f},{pos0[c,2]:.4f})")
    bag.fix_particles(particles_idx_local=corners)

    def _select_mid_top_grip():
        """현재 봉투 particle 에서 가로 중앙 + 상단 strip 선정 (grasp 직전 호출)."""
        cur = _pos_of(bag); cx, cy, cz = cur[:, 0], cur[:, 1], cur[:, 2]
        x_center = float(GRASP_XY[0])
        z_top    = float(cz.max())
        strip = (np.abs(cx - x_center) < GRIP_X_WIDTH) & (cz > z_top - GRIP_Z_FROM_TOP)
        idx = np.array([i for i in np.where(strip)[0] if i not in corners])
        if len(idx) > 0:
            sp = cur[idx]
            print(f"[grip-select] N={len(idx)}  "
                  f"X={sp[:,0].min():.4f}~{sp[:,0].max():.4f}  "
                  f"Y={sp[:,1].min():.4f}~{sp[:,1].max():.4f}  "
                  f"Z={sp[:,2].min():.4f}~{sp[:,2].max():.4f}")
        return idx

    print(f"[bag] initial grip strip preview:")
    _ = _select_mid_top_grip()
    grip_idx = np.array([], dtype=int)

    def bag_com():
        p = _pos_of(bag); v = p[~np.isnan(p).any(axis=1)]
        return v.mean(axis=0)

    cam.start_recording(); keyframes = {}; step = [0]; track = [0.0]

    def update_cam():
        tf = track[0]
        cam.set_pose(pos=_lerp(CAM_WIDE_POS, bag_com() + CAM_TRACK_OFF, tf),
                     lookat=_lerp(CAM_WIDE_LOOK, bag_com(), tf))

    def run(name, qa0, qa1, f0, f1, n, attach=False, release=False,
            drop_release=False, track_to=None):
        nonlocal grip_idx
        if attach:
            grip_idx = _select_mid_top_grip()
            bag.fix_particles_to_link(link_idx=grip_link_idx, particles_idx_local=grip_idx)
            print(f"[grasp] attach {len(grip_idx)} → {GRIP_LINK}")
        if release:
            bag.release_particle(particles_idx_local=corners)
        if drop_release:
            bag.release_particle(particles_idx_local=grip_idx)
            print("[place] released bag from gripper")
        for k in range(n):
            s = (k + 1) / n
            robot.set_dofs_position(np.concatenate([_lerp(qa0, qa1, s), [_lerp(f0, f1, s)] * 2]))
            scene.step(); step[0] += 1
            if track_to is not None:
                track[0] = _lerp(track_to[0], track_to[1], s)
            if step[0] % RENDER_EVERY == 0:
                update_cam(); img = _rgb_of(cam.render())
                if name not in keyframes:
                    keyframes[name] = img
        print(f"[phase] {name} @ {step[0]}")

    run("dropin",  Q_GRASP, Q_GRASP, FING_OPEN, FING_OPEN, N_DROP, track_to=(0.0, 0.6))
    run("close",   Q_GRASP, Q_GRASP, FING_OPEN, FING_CLOSE, N_CLOSE, track_to=(0.6, 1.0))
    run("csettle", Q_GRASP, Q_GRASP, FING_CLOSE, FING_CLOSE, N_CSET)
    run("grasp",   Q_GRASP, Q_GRASP, FING_CLOSE, FING_CLOSE, N_GRASP, attach=True, release=True)
    run("lift",    Q_GRASP, Q_LIFT,  FING_CLOSE, FING_CLOSE, N_LIFT)
    run("move",    Q_LIFT,  Q_MOVE,  FING_CLOSE, FING_CLOSE, N_MOVE)
    run("approach", Q_MOVE,     Q_APPROACH, FING_CLOSE, FING_CLOSE, N_APPROACH)
    run("settle",   Q_APPROACH, Q_APPROACH, FING_CLOSE, FING_CLOSE, N_SETTLE)
    run("descend",  Q_APPROACH, Q_DESCEND,  FING_CLOSE, FING_CLOSE, N_DESCEND)

    # ── verify: 봉투가 release 직전에 gap 중앙에 있는지 ────────────────────────
    bag_at_place = bag_com()
    ee_at_place  = _npy(ee_link.get_pos())
    err_ee_target = ee_at_place - ee_final
    err_bag_gap   = bag_at_place - gap_center
    print(f"\n[verify @ end-of-descend, before release]")
    print(f"  EE({EE_LINK_NAME}) = {ee_at_place}  (target {ee_final}, err {np.linalg.norm(err_ee_target)*1000:.1f} mm)")
    print(f"  bag com         = {bag_at_place}")
    print(f"  gap center      = {gap_center}")
    print(f"  bag - gapcenter = {err_bag_gap}  (xy_dist {np.linalg.norm(err_bag_gap[:2])*1000:.1f}mm, "
          f"dz {err_bag_gap[2]*1000:+.1f}mm)")

    run("release", Q_DESCEND, Q_DESCEND, FING_CLOSE, FING_OPEN, N_REL, drop_release=True)
    run("hold",    Q_DESCEND, Q_DESCEND, FING_OPEN, FING_OPEN, N_HOLD)

    print(f"\n[verify @ end-of-hold]  bag final com = {bag_com()}")

    cam.stop_recording(save_to_filename=MP4_PATH, fps=20)
    print(f"[saved] {MP4_PATH}")
    for nm, img in keyframes.items():
        Image.fromarray(img).save(os.path.join(OUT_DIR, f"Crusher_Samplebag_{nm}.png"))
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=True)

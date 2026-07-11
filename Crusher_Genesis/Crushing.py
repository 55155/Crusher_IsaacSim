"""
Crushing.py — 시스템 전체 통합 시뮬레이션.

구성:
  · 알루미늄 플레이트 (1m×1m×2cm, 상단 z=0, 고정)
  · M0609 6-DOF 매니퓰레이터 + OnRobot RG2 그리퍼  (robots/m0609_rg2.xml)
  · PBD 약봉투 (5면체 cloth, 상단 개방)  + 내부 Rigid 박스
  · Crusher_IsaacSim (Wall_1 슬라이드 + 크랭크슬라이더 메커니즘) ─ 로봇 정면, Wall_1 -x 향함

시퀀스:
  dropin → close → csettle → grasp → lift → move → place → release → hold
  (j2+j3+j5=2.90 고정 → EE orientation 보존)

핵심 처리:
  · Crusher MJCF 런타임 패치 (lock_crank equality 제거, weld solref/solimp 강화, ground 제거)
  · Crusher 엔티티에 decimate=False, convexify=False, surface(smooth=False) → 원본 STL 보존

출력:
  Sim_result/Crushing.mp4 + 페이즈별 키프레임 PNG
"""
import os, sys, shutil, tempfile
import xml.etree.ElementTree as ET
from datetime import datetime
import numpy as np
import trimesh as tm
from PIL import Image

# ── 시뮬 옵션 ────────────────────────────────────────────────────────────────
DT, SUBSTEPS = 1e-3, 10
RENDER_EVERY = 15

# ── 봉투 (cloth 패널) ────────────────────────────────────────────────────────
#   D(두께)는 PBD 하한 주의: 앞/뒤 패널 간격이 2*particle_size(≈5.7mm) 보다 작으면
#   파티클-파티클 충돌제약이 즉시 위반(2.1 폭발) → 6mm 가 사실상 최소.
#   사용자 요청으로 전체 스케일 ↓ (80×120 → 50×80 mm).
W, H, D = 0.05, 0.08, 0.006
NW, NH, ND = 6, 9, 2
PARTICLE_SIZE = 2.83e-3

# ── 로봇 waypoint (S=j2+j3+j5=2.90 → EE 자세 유지) ────────────────────────
# 초기 fallback 값. main() 에서 grid scan 으로 새 BAG_POS/CRUSHER_POS 에 맞춰 재계산.
Q_GRASP_INIT = np.array([0, -0.40, 1.30, 0, 2.00, 0], float)
Q_LIFT_INIT  = np.array([0, -0.11, 0.60, 0, 2.41, 0], float)
Q_MOVE_INIT  = np.array([0, -0.05, 0.85, 0, 2.10, 0], float)
FING_OPEN, FING_CLOSE = 0.04, 0.006

# ── 봉투/박스 spawn ─────────────────────────────────────────────────────────
#   바닥(z=0 알루미늄판 상단) 근처로 lower. Q_GRASP 는 main()에서 grid scan 으로 재계산.
GRASP_XY    = np.array([0.30, 0.0]); BAG_MOUTH_Z = 0.20
BAG_POS     = (GRASP_XY[0], GRASP_XY[1], BAG_MOUTH_Z - H/2)
# 파지 대상: 봉투 "가로 중앙 + 상단" 좁은 strip (3D throat 박스 대신)
GRIP_X_WIDTH    = 0.005
GRIP_Z_FROM_TOP = 0.005
GRIP_LINK       = "rg2_left"
BOX_SIZE    = (0.025, 0.004, 0.025)     # bag scale 따라 box 도 축소
BOX_RHO     = 300.0
BOX_SPAWN   = (GRASP_XY[0], GRASP_XY[1], BAG_MOUTH_Z + 0.075)

# ── 페이즈 step (DT 변경에 맞춰 절반으로) ───────────────────────────────────
N_DROP, N_CLOSE, N_CSET, N_GRASP, N_LIFT, N_MOVE, N_APPROACH, N_SETTLE, N_DESCEND, N_REL, N_HOLD = \
    500, 250, 150, 75, 450, 600, 350, 500, 350, 150, 150
# N_SETTLE: APPROACH 직후 공중에서 봉투 진동 안정화 (0.5 s)

# ── 카메라 ──────────────────────────────────────────────────────────────────
CAM_WIDE_POS  = np.array([1.15, -1.05, 1.05])
CAM_WIDE_LOOK = np.array([0.27, 0.0, 0.45])
CAM_TRACK_OFF = np.array([0.45, -0.42, 0.18])

# ── Crusher 배치 (바닥 위, 매니퓰레이터 정면) ──────────────────────────────
#   사용자 요청: 공중 부양 제거 → z=0 (알루미늄판 상단에 안착).
#   Wall_top world ≈ 0.10 (정도). Q_DESCEND 는 grid scan 으로 닿게 찾음.
CRUSHER_POS   = (0.55, 0.0, 0.0)
CRUSHER_EULER = (0.0, 0.0, 90.0)

# ── PBD 봉투 강성 (Samplebag 에서 검증: 박스 안전 + 유연성 ↑) ──────────────
STRETCH_COMPLIANCE = 1e-3
BENDING_COMPLIANCE = 1e-3

# ── 슬롯(=L2_Wall3_1 수직 back wall) 앞 placement ─────────────────────────
#   Wall_1 은 MJCF 에서 body/joint 없는 worldbody geom → link 아님, geom 으로 조회.
#   (geom 식별: geom.metadata["mesh_path"]; 월드 좌표: geom.get_AABB() = 정점기반,
#    get_pos() 는 frame origin 이라 mesh 위치 아님)
WALL_GEOM_MESH    = "L2_Wall3_1"     # 봉투가 기대는 수직 back wall mesh
EE_LINK_NAME      = "rg2_left"       # 봉투는 핑거끝(rg2_left)에 weld → 핑거를 타깃
# 슬롯 앞면(min-x) 기준 핑거 TCP 목표.
#   xy: 앞으로(−x) 살짝 띄워 벽 앞면에 정렬
#   z : 벽 상단(max-z) 기준 위로 띄운다 → 핑거가 벽면과 충돌 안 함(문제1).
#       봉투는 그 아래로 늘어져 슬롯에 들어간다.
SLOT_DX           = -0.015           # 앞으로(−x)
SLOT_DY           =  0.0
SLOT_DZ_FINAL     =  0.02            # 봉투 bottom 이 plate 근처까지 닿도록 깊게 하강
# 2-step IK(문제2): step1 = 슬롯 바로 위(APPROACH), step2 = z 만 내려 DESCEND
APPROACH_DZ       =  0.12            # APPROACH 는 최종보다 이만큼 더 위
# place 에서 EE(공구축 roll, j6)를 90° 돌려 봉투를 90° 회전 → 봉투 얇은 면을
# 슬롯의 얇은 x-gap 에 정렬(현재 봉투 두께는 y 방향 → 회전 후 x 방향).
J6_PLACE          =  np.pi / 2.0     # +90° (정렬 안 맞으면 -np.pi/2 로 부호 반전)

# ── 경로: config.json + paths.py (중앙 해석) — 위치 독립 부트스트랩 ──────────
_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
import paths

OUT_DIR = paths.SIM_RESULT
STL_PATH = os.path.join(OUT_DIR, "medicine_envelope_open.stl")
# 타임스탬프로 mp4 분리 저장 (이전 결과 비교용)
_TS      = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4_PATH = os.path.join(OUT_DIR, f"Crushing_{_TS}.mp4")
PLATE_PATH = paths.ALUMINUM_PLATE
ROBOT_MJCF = paths.ROBOT_M0609_RG2
CRUSHER_SRC_XML = paths.MJCF_MAIN


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


WALL_GEOMS_TO_ENABLE = {"L1_Wall1_1", "L1_Wall2_1", "L2_Wall3_1"}
L7_LINK3_COM = "0.006 0 -0.005"


def patch_crusher_mjcf(src, dst,
                        eq_solref="0.0002 50",
                        eq_solimp="0.999 0.99999 1e-5"):
    """원본 Crusher MJCF → Genesis 호환 사본.

    - <equality><joint> (lock_crank polycoef): 제거
    - <equality><weld> 유지, solref/solimp 매우 빡빡한 값으로 교체
        timeconst=0.2ms, dampratio=50, solimp 최대 impedance
    - <geom name="ground">: 알루미늄 플레이트와 중복이라 제거
    - 벽 geom (L1_Wall1_1, L1_Wall2_1, L2_Wall3_1) 의 충돌 활성화
    - L7_Link3_1 의 dubious CoM 을 bbox 중심으로 교체 (URDF 변환 오류 보정)
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


# RG2 핑거 충돌 활성화 — 원본 m0609_rg2.xml 에서 contype="0" 라 PBD coupler 가
# 무시. 런타임 패치로 contype/conaffinity 속성 제거 → default 0xFFFF (충돌 ON).
RG2_GEOMS_TO_ENABLE = {"rg2_finger", "rg2_hand"}


def patch_robot_mjcf(src, dst):
    """m0609_rg2.xml → RG2 핑거/손 collision 활성화 사본."""
    tree = ET.parse(src); root = tree.getroot()
    wb = root.find("worldbody")
    if wb is not None:
        for g in wb.iter("geom"):
            if g.get("mesh") in RG2_GEOMS_TO_ENABLE:
                g.attrib.pop("contype", None)
                g.attrib.pop("conaffinity", None)
    tree.write(dst)


def _prepare_robot_mjcf():
    """robots/m0609_rg2.xml 디렉터리 복사 + 패치본 생성."""
    src_xml = ROBOT_MJCF
    src_dir = os.path.dirname(src_xml)
    tmp_dir = tempfile.mkdtemp(prefix="m0609_mjcf_")
    # robots/ 하위 전체 복사 (m0609/assets/*, rg2/assets/* 등 mesh 의존)
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


# worldbody 정적 벽 geom 공통 변환: pos="0 0 0", quat="0.5 0.5 0.5 0.5"(→ R 아래), scale 1e-3
_R_GEOM_HALF = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])  # quat(.5,.5,.5,.5)


def wall_geom_world_aabb(mesh_name):
    """worldbody 정적 벽 geom 의 월드 AABB (해석적, build 불필요).

    Genesis 가 정적 충돌 geom 을 link 당 1개로 병합해 개별 mesh live 조회가 안 되므로,
    geom MJCF 변환(pos0, quat .5.5.5.5, scale 1e-3) + 엔티티 변환(CRUSHER_POS, yaw)으로
    STL 정점을 직접 월드로 보내 min/max 를 구한다. (벽은 완전 정적 → 해석=실측)
    """
    yaw = np.radians(CRUSHER_EULER[2])
    R_e = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                    [np.sin(yaw),  np.cos(yaw), 0.],
                    [0., 0., 1.]])
    v = tm.load(os.path.join(paths.MJCF_DIR, f"{mesh_name}.stl")).vertices * 0.001
    w = np.array(CRUSHER_POS) + (R_e @ (_R_GEOM_HALF @ v.T)).T
    return w.min(axis=0), w.max(axis=0)


def main(use_viewer: bool = True):
    print("="*60); print(f" Crushing — full pick & move + Crusher (viewer={use_viewer})"); print("="*60)
    make_bag()
    crusher_xml = _prepare_crusher_mjcf()
    print(f"[crusher] patched MJCF → {crusher_xml}")
    robot_xml = _prepare_robot_mjcf()
    print(f"[robot]   patched MJCF → {robot_xml}  (RG2 핑거 충돌 ON)")

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

    # 알루미늄 플레이트 — 2×2 그리드 (각 1m×1m). 로봇 정면 +x, 좌우 ±y 까지 작업면 확장
    plate_positions = [
        ( 0.5, -0.5, 0.0),   # 정면 좌
        ( 0.5,  0.5, 0.0),   # 정면 우
        (-0.5, -0.5, 0.0),   # 뒤 좌
        (-0.5,  0.5, 0.0),   # 뒤 우
    ]
    for p in plate_positions:
        scene.add_entity(
            gs.morphs.Mesh(file=PLATE_PATH, fixed=True, pos=p),
            material=gs.materials.Rigid(),
            surface=gs.surfaces.Default(color=(0.82, 0.82, 0.85), metallic=0.85, roughness=0.3),
        )
    # M0609 + RG2 (핑거 충돌 활성화 패치본)
    robot = scene.add_entity(gs.morphs.MJCF(file=robot_xml, decimate=False))
    # Crusher (정면, Wall_1 -x 향함, faceted 정밀 렌더)
    crusher = scene.add_entity(
        gs.morphs.MJCF(file=crusher_xml, pos=CRUSHER_POS, euler=CRUSHER_EULER,
                       decimate=False, convexify=False),
        surface=gs.surfaces.Default(smooth=False),
    )
    # PBD 약봉투 — Samplebag 검증된 유연 강성
    bag = scene.add_entity(
        material=gs.materials.PBD.Cloth(
            stretch_compliance=STRETCH_COMPLIANCE,
            bending_compliance=BENDING_COMPLIANCE,
        ),
        morph=gs.morphs.Mesh(file=STL_PATH, scale=1.0, pos=BAG_POS, euler=(90, 0, 0)),
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.7, roughness=0.9, double_sided=True),
    )
    # 내용물 박스
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

    # ── Wall_1(=L2_Wall3_1) 슬롯 위치 ───────────────────────────────────────
    #   이 벽은 body/joint 없이 worldbody 에 박힌 정적 geom 이라 link.get_pos() 로
    #   안 잡힌다. 게다가 Genesis 는 같은 링크(world)의 충돌 mesh geom 들을 1개로
    #   "병합"하므로 개별 mesh 를 entity.geoms 에서 live 조회할 수도 없다.
    #   → 완전 정적 geom 이므로 MJCF geom 변환(pos0, quat .5.5.5.5, scale 1e-3) +
    #     엔티티 변환(CRUSHER_POS, yaw) 으로 STL 정점을 직접 월드로 보내 AABB 계산.
    ee_link = robot.get_link(EE_LINK_NAME)
    w_lo, w_hi = wall_geom_world_aabb(WALL_GEOM_MESH)
    w_center = (w_lo + w_hi) / 2.0
    # 슬롯 앞면(min-x) xy + 벽 상단(max-z) 위로 띄운 z → 핑거가 벽면과 충돌 안 함(문제1)
    slot_xy   = np.array([w_lo[0] + SLOT_DX, w_center[1] + SLOT_DY])
    ee_final    = np.array([slot_xy[0], slot_xy[1], w_hi[2] + SLOT_DZ_FINAL])   # 최종(하강 후)
    ee_approach = ee_final + np.array([0.0, 0.0, APPROACH_DZ])                  # 슬롯 바로 위
    print(f"\n[wall] geom '{WALL_GEOM_MESH}'  world AABB lo={np.round(w_lo,4)} hi={np.round(w_hi,4)}")
    print(f"[wall] wall_top_z={w_hi[2]:.4f}  ee_final={np.round(ee_final,4)}  "
          f"ee_approach={np.round(ee_approach,4)}")

    # Genesis IK 가 이 robot/target 조합에서 비정상 — manual brute search 로 대체.
    # j2+j3+j5=2.90 invariant 유지 (EE 수직 자세 보존), j4=0 고정.
    # j6 = J6_PLACE 로 공구축 roll 을 줘서 봉투를 90° 회전(문제1). j6 가 rg2_left 를
    # 공구축 둘레로 돌리므로 scan 이 그 오프셋까지 반영해 j1,j2,j3 를 찾는다.
    def _find_q_place(target, j6=0.0):
        # 1단: 거친 grid scan
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
        # 2단: 1단 결과 주변에서 미세 scan (해상도 ↑)
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

    # ── 모든 waypoint Q 를 grid scan 으로 동적 계산 (새 BAG/CRUSHER 위치 대응) ──
    # GRASP: 봉투 mouth 바로 위 (j6=0, 자연 자세)
    target_grasp = np.array([GRASP_XY[0], GRASP_XY[1], BAG_MOUTH_Z + 0.02])
    Q_GRASP, err_g, ee_g = _find_q_place(target_grasp, j6=0.0)
    # LIFT: GRASP 에서 z +20cm
    target_lift  = target_grasp + np.array([0.0, 0.0, 0.20])
    Q_LIFT,  err_l, ee_l = _find_q_place(target_lift, j6=0.0)
    # MOVE: GRASP 와 APPROACH 의 중간 (위로 휘둘러 이동)
    target_move  = (target_lift + ee_approach) / 2.0
    Q_MOVE,  err_m, ee_m = _find_q_place(target_move, j6=J6_PLACE/2.0)
    # APPROACH / DESCEND: 슬롯 위 / 슬롯 안 (j6=π/2, 봉투 90° 회전)
    Q_APPROACH, err_a, ee_a = _find_q_place(ee_approach, j6=J6_PLACE)
    Q_DESCEND,  err_d, ee_d = _find_q_place(ee_final, j6=J6_PLACE)
    print(f"\n[waypoint scan]")
    print(f"  GRASP   q={np.round(Q_GRASP,3)} EE={np.round(ee_g,4)} (target {np.round(target_grasp,4)} err {err_g*1000:.1f}mm)")
    print(f"  LIFT    q={np.round(Q_LIFT,3)} EE={np.round(ee_l,4)} (target {np.round(target_lift,4)} err {err_l*1000:.1f}mm)")
    print(f"  MOVE    q={np.round(Q_MOVE,3)} EE={np.round(ee_m,4)} (target {np.round(target_move,4)} err {err_m*1000:.1f}mm)")
    print(f"  APPROACH q={np.round(Q_APPROACH,3)} EE={np.round(ee_a,4)} (target {np.round(ee_approach,4)} err {err_a*1000:.1f}mm)")
    print(f"  DESCEND  q={np.round(Q_DESCEND,3)} EE={np.round(ee_d,4)} (target {np.round(ee_final,4)} err {err_d*1000:.1f}mm)")

    # 다시 시작 자세 (FING_OPEN)
    robot.set_dofs_position(np.concatenate([Q_GRASP, [FING_OPEN, FING_OPEN]]))

    pos0 = _pos_of(bag); z, x, y = pos0[:, 2], pos0[:, 0], pos0[:, 1]
    band = np.where(z >= np.quantile(z, 0.92))[0]
    # 4개 코너: (xmin,ymin), (xmax,ymin), (xmin,ymax), (xmax,ymax)
    # 기존 argmin(-x+y) 는 argmax(x-y) 와 동일 → 중복. 수정: argmax(y-x).
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
        """현재 상태의 봉투 particle 에서 가로 중앙 + 상단 strip 선정.
        grasp 직전에 호출 → dropin/close 변형 후 실제 위치 기반 선정."""
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

    # 초기 진단 — sim 시작 전 grip 후보 위치
    print(f"[bag] initial grip strip preview:")
    _ = _select_mid_top_grip()
    grip_idx = np.array([], dtype=int)  # 실제 선정은 grasp phase 직전에

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
            # grasp 직전에 동적 재선정 — dropin/close 후 실제 봉투 상태 기준
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
    # 3-step: 슬롯 바로 위 → 공중 정지(봉투 안정화) → z 만 수직 하강
    run("approach", Q_MOVE,     Q_APPROACH, FING_CLOSE, FING_CLOSE, N_APPROACH)
    run("settle",   Q_APPROACH, Q_APPROACH, FING_CLOSE, FING_CLOSE, N_SETTLE)
    run("descend",  Q_APPROACH, Q_DESCEND,  FING_CLOSE, FING_CLOSE, N_DESCEND)

    # ── Phase A verify: 봉투가 release 직전에 Wall_1 입구에 있는지 ──────────
    bag_at_place = bag_com()
    ee_at_place  = _npy(ee_link.get_pos())
    # 벽은 정적이라 front_face 동일 (해석값 재사용)
    wall_front = np.array([w_lo[0], w_center[1], w_center[2]])
    err_ee_target = ee_at_place - ee_final
    err_bag_wall  = bag_at_place - wall_front
    print(f"\n[verify @ end-of-descend, before release]")
    print(f"  EE({EE_LINK_NAME}) = {ee_at_place}  (target {ee_final}, err {np.linalg.norm(err_ee_target)*1000:.1f} mm)")
    print(f"  bag com         = {bag_at_place}")
    print(f"  wall front_face = {wall_front}")
    print(f"  bag - wallfront = {err_bag_wall}  (xy_dist {np.linalg.norm(err_bag_wall[:2])*1000:.1f}mm, "
          f"dz {err_bag_wall[2]*1000:+.1f}mm)")

    run("release", Q_DESCEND, Q_DESCEND, FING_CLOSE, FING_OPEN, N_REL, drop_release=True)
    run("hold",    Q_DESCEND, Q_DESCEND, FING_OPEN, FING_OPEN, N_HOLD)

    print(f"\n[verify @ end-of-hold]  bag final com = {bag_com()}  (will fall w/o holder clamp)")

    cam.stop_recording(save_to_filename=MP4_PATH, fps=20)
    print(f"[saved] {MP4_PATH}")
    for nm, img in keyframes.items():
        Image.fromarray(img).save(os.path.join(OUT_DIR, f"Crushing_{nm}.png"))
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=True)

"""
Crusher_Samplebag.py — Crusher + Samplebag(내부 박스) 단독 삽입+클램프+크랭크 공정.

매니퓰레이터/RG2 제거. 봉투를 carrier (invisible kinematic rigid) 에 weld 해서
gap 위에서 시작 → 천천히 하강 → 목표(=Wall_back ↔ Left_Wall 사이) 도달.
이후 Motor2 가 Left_Wall 을 닫고, Motor1 크랭크가 동작해 박스를 타격.

시퀀스:
  1. openwall : Left_Wall 0 → -7mm 개방. 완전히 열릴 때까지 다른 동작 정지(봉투 매단 채 대기)
  2. bagsettle: 매달린 봉투 안정화 (박스 옆에 대기)
  3. dropbox  : 박스를 mouth 위로 옮겨 매달린 봉투 안으로 낙하
  4. descend  : carrier 가 봉투(+박스)를 start_pos → target_pos(슬롯) 로 선형 하강
  5. warmup   : Motor1 크랭크 0 → -π/2 (start angle) 까지 PD position 램프
  6. clamp    : Motor2 Left_Wall +5 mm/s 슬라이드 (back wall 쪽으로 닫힘)
  7. crank    : Motor1 +π rad/s 등속 회전 (분쇄)

출력:
  Sim_result/Crusher_Samplebag_<ts>_side_xz.mp4 (측면 x-z) + _top_xy.mp4 (평면 x-y)
"""
import os, sys, shutil, tempfile
import xml.etree.ElementTree as ET
from datetime import datetime
import numpy as np
import trimesh as tm

# Windows cp949 한글 깨짐 방지
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

# ── 시뮬 옵션 ────────────────────────────────────────────────────────────────
DT, SUBSTEPS = 1e-3, 10
RENDER_EVERY = 15

# ── 샘플백 (PBD cloth) ──────────────────────────────────────────────────────
# 원래 medicine envelope 스케일 (80×120 mm). D 만 PBD 안정성 위해 6 mm 유지
# (원본 10 mm → 6 mm; 2·particle_size=5.66 mm 보다 큼).
W, H, D = 0.08, 0.12, 0.006     # 80 × 120 × 6 mm
NW, NH, ND = 16, 24, 2
PARTICLE_SIZE = 2.83e-3
STRETCH_COMPLIANCE = 1e-3
BENDING_COMPLIANCE = 1e-3

# ── 박스 (봉투 내용물) ──────────────────────────────────────────────────────
BOX_SIZE = (0.015, 0.004, 0.015)
BOX_RHO  = 300.0

# ── Crusher 배치 ────────────────────────────────────────────────────────────
CRUSHER_POS   = (0.55, 0.0, 0.0)
CRUSHER_EULER = (0.0, 0.0, 90.0)

# ── 타깃 / 시작 오프셋 ──────────────────────────────────────────────────────
SLOT_DX_FRONT  =  0.000   # gap 중심 기준 x 오프셋 (0 = Wall3↔Left_Wall 정중앙)
SLOT_DZ_START  =  0.20    # target 보다 20 cm 위에서 시작
SLOT_DZ_FINAL  =  0.00    # 봉투 mouth 이 wall_top 과 같은 높이

# ── 크랭크 / 벽 PD 게인 + 동작 파라미터 ─────────────────────────────────────
CRANK_KP, CRANK_KV = 2000.0, 100.0
WALL_KP,  WALL_KV  = 5000.0, 500.0
CRANK_START_Q = -np.pi / 2      # 크랭크 초기 -90°
CRANK_OMEGA   =  np.pi          # 0.5 rev/s (분쇄)
WALL_VEL      =  0.005          # 5 mm/s (천천히 닫힘)
WALL_OFFSET   = -0.007          # Left_Wall 초기 -7 mm (gap 확장; 봉투 투입용)

# ── 페이즈 step 수 ──────────────────────────────────────────────────────────
N_OPENWALL  = 800   # 0.8 s  Left_Wall 0 → -7mm 개방 + 안착 (이 동안 봉투/박스 정지)
N_BAGSETTLE = 800   # 0.8 s  매달린 봉투 안정화 (박스 투입 전)
N_DROPBOX   = 1200  # 1.2 s  매달린 봉투 안으로 박스 낙하 + 안정화
N_DESCEND   = 1500  # 1.5 s  봉투(+박스) 슬롯으로 하강
N_WARMUP    = 500   # 0.5 s
N_CLAMP     = 1000  # 1.0 s (5 mm/s × 1s = 5 mm 슬라이드)
N_CRANK     = 4000  # 4.0 s (4 회전)

# ── 카메라 (슬롯 줌 인: 측면 y-z + 평면 x-y) ───────────────────────────────
CAM_RES   = (960, 720)
CAM_FOV   = 45
CAM_DIST  = 0.40              # 슬롯 중심 ↔ 카메라 거리
# 인터랙티브 viewer 용 광각
CAM_WIDE_POS  = np.array([0.85, -0.45, 0.45])
CAM_WIDE_LOOK = np.array([0.50, 0.0, 0.08])

# ── 정적 벽 mesh (Wall_1 ↔ Left_Wall 사이의 좁은 gap 이 타깃) ─────────────
WALL_BACK_MESH    = "L2_Wall3_1"   # Left_Wall 과 마주보는 수직 back wall (L1_Wall1_1 은 바닥 플레이트라 오류)
WALL_LEFT_MESH    = "L2_Left_Wall1_1"
LEFTWALL_BODY_POS = (-0.017802, 0.286278, 0.016542)
LEFTWALL_GEOM_POS = (-0.286278, -0.016542, 0.017802)

# ── 경로 ────────────────────────────────────────────────────────────────────
_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
STL_PATH = os.path.join(OUT_DIR, "crusher_samplebag_open.stl")
_TS      = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4_SIDE = os.path.join(OUT_DIR, f"Crusher_Samplebag_{_TS}_side_xz.mp4")
MP4_TOP  = os.path.join(OUT_DIR, f"Crusher_Samplebag_{_TS}_top_xy.mp4")
CRUSHER_SRC_XML = os.path.join(_DIR, "MJCF", "Crusher_IsaacSim_colored.xml")

# ── Crusher MJCF 패치 옵션 ──────────────────────────────────────────────────
WALL_GEOMS_TO_ENABLE = {"base_link", "L1_Wall1_1", "L1_Wall2_1", "L2_Wall3_1"}
L7_LINK3_COM = "0.006 0 -0.005"


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


def patch_crusher_mjcf(src, dst,
                        eq_solref="0.0002 50",
                        eq_solimp="0.999 0.99999 1e-5"):
    """원본 Crusher MJCF → Genesis 호환 사본."""
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


def _npy(x): return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)
def _pos_of(b):
    x = _npy(b.get_particles_pos())
    return x[0] if x.ndim == 3 else x
def _rgb_of(r):
    a = r[0] if isinstance(r, (tuple, list)) else r
    a = _npy(a)
    return a[..., :3].astype("uint8")


# 정적 벽 geom 변환: quat=(.5,.5,.5,.5) → 아래 행렬, scale 1e-3
_R_GEOM_HALF = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])


def crusher_mesh_world_aabb(mesh_name, body_pos=(0., 0., 0.), geom_pos=(0., 0., 0.)):
    yaw = np.radians(CRUSHER_EULER[2])
    R_e = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                    [np.sin(yaw),  np.cos(yaw), 0.],
                    [0., 0., 1.]])
    v = tm.load(os.path.join(_DIR, "MJCF", f"{mesh_name}.stl")).vertices * 0.001
    local      = np.asarray(geom_pos) + v
    in_crusher = np.asarray(body_pos) + (_R_GEOM_HALF @ local.T).T
    w          = np.array(CRUSHER_POS) + (R_e @ in_crusher.T).T
    return w.min(axis=0), w.max(axis=0)


def main(use_viewer: bool = True):
    print("="*60)
    print(f" Crusher_Samplebag — 봉투 삽입 + 클램프 + 크랭크 (viewer={use_viewer})")
    print("="*60)
    make_bag()
    crusher_xml = _prepare_crusher_mjcf()
    print(f"[crusher] patched MJCF → {crusher_xml}")

    import genesis as gs
    _backend = gs.metal if sys.platform == "darwin" else gs.cuda
    gs.init(backend=_backend, logging_level="warning")

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
            camera_fov=CAM_FOV, max_FPS=60)
    scene = gs.Scene(**scene_kwargs)

    # 알루미늄 plate 2×2 그리드 (작업면)
    PLATE_PATH = os.path.join(_DIR, "robots/assets/aluminum_plate.stl")
    for p in [(0.5, -0.5, 0), (0.5, 0.5, 0), (-0.5, -0.5, 0), (-0.5, 0.5, 0)]:
        scene.add_entity(
            gs.morphs.Mesh(file=PLATE_PATH, fixed=True, pos=p),
            material=gs.materials.Rigid(),
            surface=gs.surfaces.Default(color=(0.82, 0.82, 0.85), metallic=0.85, roughness=0.3),
        )
    # Crusher
    crusher = scene.add_entity(
        gs.morphs.MJCF(file=crusher_xml, pos=CRUSHER_POS, euler=CRUSHER_EULER,
                       decimate=False, convexify=False),
        surface=gs.surfaces.Default(smooth=False),
    )

    # ── gap 위치 해석 (build 전) ────────────────────────────────────────────
    # 진단: 모든 정적 벽 AABB 출력 (Wall_1 / Wall_2 / Wall_3 / Left_Wall)
    for nm in ("L1_Wall1_1", "L1_Wall2_1", "L2_Wall3_1"):
        lo, hi = crusher_mesh_world_aabb(nm)
        print(f"[wall-aabb] {nm:<12} x=[{lo[0]:+.4f},{hi[0]:+.4f}] "
              f"y=[{lo[1]:+.4f},{hi[1]:+.4f}] z=[{lo[2]:+.4f},{hi[2]:+.4f}]")
    lw_lo, lw_hi = crusher_mesh_world_aabb(WALL_LEFT_MESH, LEFTWALL_BODY_POS, LEFTWALL_GEOM_POS)
    print(f"[wall-aabb] {WALL_LEFT_MESH:<12} x=[{lw_lo[0]:+.4f},{lw_hi[0]:+.4f}] "
          f"y=[{lw_lo[1]:+.4f},{lw_hi[1]:+.4f}] z=[{lw_lo[2]:+.4f},{lw_hi[2]:+.4f}]")

    wb_lo, wb_hi = crusher_mesh_world_aabb(WALL_BACK_MESH)
    wl_lo, wl_hi = lw_lo, lw_hi
    gap_lo_x, gap_hi_x = sorted([wb_hi[0], wl_lo[0]])
    gap_cx = (gap_lo_x + gap_hi_x) / 2.0
    gap_width = gap_hi_x - gap_lo_x
    y_lo = max(wb_lo[1], wl_lo[1]); y_hi = min(wb_hi[1], wl_hi[1])
    gap_cy = (y_lo + y_hi) / 2.0
    wall_top_z = max(wb_hi[2], wl_hi[2])

    # 목표 = 봉투 중심이 wall_top 살짝 아래 (봉투 mouth ~ wall_top 정렬)
    target_x = gap_cx + SLOT_DX_FRONT         # Wall3↔Left_Wall gap 중심
    target_y = gap_cy
    target_z = wall_top_z + SLOT_DZ_FINAL
    # 시작 = target 위 + 동일 xy
    start_x  = target_x
    start_y  = target_y
    start_z  = target_z + SLOT_DZ_START

    print(f"\n[gap]    {WALL_BACK_MESH}.+x={wb_hi[0]:.4f}  Left_Wall.-x={wl_lo[0]:.4f}  "
          f"width={gap_width*1000:.2f}mm  wall_top_z={wall_top_z:.4f}")
    print(f"[bag]    start = ({start_x:.4f}, {start_y:.4f}, {start_z:.4f})")
    print(f"[bag]    target= ({target_x:.4f}, {target_y:.4f}, {target_z:.4f})")

    # ── 봉투 회전 후 x-축 두께 = D (6 mm). gap 통과 가능한지 사전 검증 ─────
    # Left_Wall 을 WALL_OFFSET (음수) 만큼 미리 이동 → 효과 gap = static + |WALL_OFFSET|
    bag_thickness_x = D                       # euler=(90,0,90) 후 D 가 x 방향
    bag_span_y      = W                       # W 가 y 방향
    eff_gap = gap_width - WALL_OFFSET         # WALL_OFFSET<0 이면 gap 확장
    margin_x = eff_gap - bag_thickness_x
    min_clearance = 2 * PARTICLE_SIZE         # PBD 충돌 솔버 여유
    fit_ok = margin_x > min_clearance
    print(f"[fit]    static_gap={gap_width*1000:.2f}mm  wall_offset={WALL_OFFSET*1000:+.2f}mm"
          f"  eff_gap={eff_gap*1000:.2f}mm")
    print(f"[fit]    bag_thickness_x={bag_thickness_x*1000:.2f}mm  margin={margin_x*1000:+.2f}mm"
          f"  required>={min_clearance*1000:.2f}mm  → {'PASS' if fit_ok else 'FAIL'}")
    if not fit_ok:
        print(f"[fit][WARN] gap 가 좁아 봉투 통과 불가. D 또는 WALL_OFFSET 조정 필요.")
    # mouth (현 회전 후 x-y 평면) vs 박스 단면
    box_x, box_y = BOX_SIZE[0], BOX_SIZE[1]
    if box_x > bag_thickness_x:
        print(f"[fit][WARN] BOX_SIZE[0]={box_x*1000:.1f}mm > bag mouth x={bag_thickness_x*1000:.1f}mm"
              f" — 박스가 봉투 mouth 통과 불가. BOX_SIZE 재조정 필요.")
    if box_y > bag_span_y:
        print(f"[fit][WARN] BOX_SIZE[1]={box_y*1000:.1f}mm > bag mouth y={bag_span_y*1000:.1f}mm")
    # 봉투 깊이 (H) vs 벽 깊이 — 봉투 bottom 이 wall bottom 보다 낮으면 plate 와 충돌
    wall_depth = wall_top_z - wb_lo[2]
    bag_bottom_z = target_z - H
    print(f"[depth]  wall(z)=[{wb_lo[2]:.4f}, {wall_top_z:.4f}]  depth={wall_depth*1000:.1f}mm  "
          f"bag(H)={H*1000:.0f}mm  bag_bottom_z={bag_bottom_z:.4f}")
    if bag_bottom_z < wb_lo[2]:
        print(f"[depth][WARN] bag_bottom({bag_bottom_z:.4f}) < wall_bottom({wb_lo[2]:.4f})"
              f" — 봉투가 벽 아래로 {(wb_lo[2]-bag_bottom_z)*1000:.1f}mm 비져나옴. "
              f"SLOT_DZ_FINAL 을 +{(wb_lo[2]-bag_bottom_z+0.005)*1000:.0f}mm 정도 올리거나 H 축소 검토.")

    # ── 봉투, 박스, carrier ───────────────────────────────────────────────
    bag_pos_init = (start_x, start_y, start_z - H/2)   # 봉투 중심
    bag = scene.add_entity(
        material=gs.materials.PBD.Cloth(
            stretch_compliance=STRETCH_COMPLIANCE,
            bending_compliance=BENDING_COMPLIANCE,
        ),
        morph=gs.morphs.Mesh(file=STL_PATH, scale=1.0, pos=bag_pos_init, euler=(90, 0, 90)),
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.7, roughness=0.9, double_sided=True),
    )
    # 박스 — 봉투가 슬롯에 안정화될 때까지 옆에 파킹(set_pos 로 hold), 이후 mouth 위로
    #   옮겨 낙하시킨다. dynamic(fixed=False) 이라 매 step set_pos 로 붙들어 둠.
    box_park   = (target_x, target_y - 0.20, target_z + 0.10)   # 슬롯에서 y 로 비켜둔 대기 위치
    box_drop_p = (start_x, start_y, start_z + 0.03)             # 매달린 봉투 mouth 바로 위
    box = scene.add_entity(
        material=gs.materials.Rigid(rho=BOX_RHO),
        morph=gs.morphs.Box(size=BOX_SIZE, pos=box_park, fixed=False),
        surface=gs.surfaces.Default(color=(0.85, 0.35, 0.25)),
    )
    # carrier — 봉투 corners 가 부착될 invisible kinematic rigid.
    #   fixed=True 라서 gravity 영향 없음. set_pos 로 위치 갱신.
    carrier_init = (start_x, start_y, start_z)
    carrier = scene.add_entity(
        material=gs.materials.Rigid(),
        morph=gs.morphs.Box(size=(0.005, 0.005, 0.005),
                            pos=carrier_init, fixed=True),
        surface=gs.surfaces.Default(color=(0.2, 0.8, 0.2), opacity=0.4),
    )

    # 슬롯 중심 기준 2개 카메라:
    #   cam_side : -y 에서 +y 로 바라봄 → 이미지 평면 (x, z) [측면, 틈 사이 하강이 보임]
    #   cam_top  : +z 에서 -z 로 내려다봄 → 이미지 평면 (x, y) [평면]
    slot_center = np.array([gap_cx, gap_cy, wall_top_z])
    cam_side = scene.add_camera(
        res=CAM_RES, pos=tuple(slot_center + np.array([0.0, -CAM_DIST, 0.0])),
        lookat=tuple(slot_center), up=(0.0, 0.0, 1.0), fov=CAM_FOV, GUI=False)
    cam_top = scene.add_camera(
        res=CAM_RES, pos=tuple(slot_center + np.array([0.0, 0.0, CAM_DIST])),
        lookat=tuple(slot_center), up=(0.0, 1.0, 0.0), fov=CAM_FOV, GUI=False)
    print(f"[cam] slot_center={slot_center}  side(x,z)-y  top(x,y)+z  dist={CAM_DIST}")
    scene.build(n_envs=0)

    # ── 봉투 corner 4개 식별 + carrier 에 weld ──
    pos0 = _pos_of(bag); z, x, y = pos0[:, 2], pos0[:, 0], pos0[:, 1]
    band = np.where(z >= np.quantile(z, 0.92))[0]
    corners = list({int(i) for i in [
        band[np.argmin(x[band] + y[band])], band[np.argmax(x[band] - y[band])],
        band[np.argmax(y[band] - x[band])], band[np.argmax(x[band] + y[band])]]})
    print(f"\n[bag] N={pos0.shape[0]}  corners={len(corners)}  "
          f"bbox X={x.min():.3f}~{x.max():.3f}  Z={z.min():.3f}~{z.max():.3f}")

    carrier_link_idx = carrier.links[0].idx
    bag.fix_particles_to_link(link_idx=carrier_link_idx, particles_idx_local=corners)
    print(f"[attach] {len(corners)} corners → carrier link (idx={carrier_link_idx})")

    # ── Crusher 모터 식별 + PD 게인 ──
    crusher_joints = {j.name: j for j in crusher.joints if j.name}

    def _scalar_dof(name):
        d = crusher_joints[name].dofs_idx_local
        return d[0] if isinstance(d, (list, tuple, np.ndarray)) else d

    crank_dof = _scalar_dof("L3_Bevel_GearBox_1_L4_Shaft_1")
    wall_dof  = _scalar_dof("L1_Guide1_1_L2_Left_Wall1_1")
    crusher.set_dofs_kp(np.array([CRANK_KP]), dofs_idx_local=[crank_dof])
    crusher.set_dofs_kv(np.array([CRANK_KV]), dofs_idx_local=[crank_dof])
    crusher.set_dofs_kp(np.array([WALL_KP]),  dofs_idx_local=[wall_dof])
    crusher.set_dofs_kv(np.array([WALL_KV]),  dofs_idx_local=[wall_dof])
    print(f"[ctrl] crank DOF #{crank_dof}: kp={CRANK_KP}, kv={CRANK_KV}")
    print(f"[ctrl] wall  DOF #{wall_dof}: kp={WALL_KP}, kv={WALL_KV}")

    # Left_Wall 은 닫힌(0) 상태에서 시작 → Phase 1 에서 천천히 개방한다.
    # (이전엔 init 에서 즉시 텔레포트 개방 + 봉투 하강이 동시에 일어나 순서가 섞였음)
    crusher.set_dofs_position(np.array([0.0]), dofs_idx_local=[wall_dof])
    wq0 = _npy(crusher.get_dofs_position())[wall_dof]
    print(f"[init] Left_Wall start = {wq0*1000:+.2f} mm (closed)  "
          f"target open = {WALL_OFFSET*1000:+.2f} mm → eff gap {(gap_width - WALL_OFFSET)*1000:.2f} mm")

    cam_side.start_recording()
    cam_top.start_recording()
    step = [0]

    def bag_com():
        p = _pos_of(bag); v = p[~np.isnan(p).any(axis=1)]
        return v.mean(axis=0)

    def render_tick():
        if step[0] % RENDER_EVERY == 0:
            cam_side.render()
            cam_top.render()

    start_p = np.array([start_x,  start_y,  start_z])
    final_p = np.array([target_x, target_y, target_z])
    box_park_p = np.array(box_park)

    # ── Phase 1: openwall (Left_Wall 0 → -7mm 개방, 완전히 열릴 때까지 대기) ──
    #   봉투는 carrier 가 start 위치에 매달아 둠, 박스는 옆에 hold → 벽만 움직인다.
    print(f"\n[phase] 1/7 openwall ({N_OPENWALL*DT:.1f} s) — Left_Wall 0 → {WALL_OFFSET*1000:+.0f}mm 개방 (다른 동작 정지)")
    for k in range(N_OPENWALL):
        s = (k + 1) / N_OPENWALL
        wq_target = WALL_OFFSET * s
        crusher.control_dofs_position(np.array([wq_target]), dofs_idx_local=[wall_dof])
        crusher.control_dofs_position(np.array([0.0]), dofs_idx_local=[crank_dof])
        carrier.set_pos(start_p, zero_velocity=True)           # 봉투 매단 채 정지
        box.set_pos(box_park_p, zero_velocity=True)            # 박스 대기
        scene.step(); step[0] += 1
        render_tick()
    wq = _npy(crusher.get_dofs_position())[wall_dof]
    print(f"  [openwall] wall_pos={wq*1000:+.2f} mm  (target {WALL_OFFSET*1000:+.0f}mm)")

    # ── Phase 2: bagsettle (매달린 봉투 안정화, 벽 개방 유지) ──
    print(f"[phase] 2/7 bagsettle ({N_BAGSETTLE*DT:.1f} s) — 매달린 봉투 안정화")
    for _ in range(N_BAGSETTLE):
        crusher.control_dofs_position(np.array([WALL_OFFSET]), dofs_idx_local=[wall_dof])
        crusher.control_dofs_position(np.array([0.0]), dofs_idx_local=[crank_dof])
        carrier.set_pos(start_p, zero_velocity=True)
        box.set_pos(box_park_p, zero_velocity=True)
        scene.step(); step[0] += 1
        render_tick()
    bc = bag_com()
    print(f"  [bagsettle] bag_com=({bc[0]:.3f},{bc[1]:.3f},{bc[2]:.3f})")

    # ── Phase 3: dropbox (매달린 봉투 안으로 박스 낙하) ──
    print(f"[phase] 3/7 dropbox ({N_DROPBOX*DT:.1f} s) — 박스를 mouth 위로 옮겨 낙하")
    box.set_pos(np.array(box_drop_p), zero_velocity=True)      # mouth 위로 1회 이동 후 자유낙하
    for _ in range(N_DROPBOX):
        crusher.control_dofs_position(np.array([WALL_OFFSET]), dofs_idx_local=[wall_dof])
        crusher.control_dofs_position(np.array([0.0]), dofs_idx_local=[crank_dof])
        carrier.set_pos(start_p, zero_velocity=True)           # 봉투 매단 채 정지
        scene.step(); step[0] += 1
        render_tick()
    bp = _npy(box.get_pos()); bc = bag_com()
    print(f"  [dropbox] box=({bp[0]:.3f},{bp[1]:.3f},{bp[2]:.3f})  bag_com_z={bc[2]:.4f}")

    # ── Phase 4: descend (봉투+박스를 슬롯 안으로 하강) ──
    print(f"[phase] 4/7 descend ({N_DESCEND*DT:.1f} s) — 봉투(+박스)를 슬롯으로 하강")
    for k in range(N_DESCEND):
        s = (k + 1) / N_DESCEND
        cur = tuple(start_p + (final_p - start_p) * s)
        carrier.set_pos(np.array(cur), zero_velocity=True)
        crusher.control_dofs_position(np.array([WALL_OFFSET]), dofs_idx_local=[wall_dof])
        crusher.control_dofs_position(np.array([0.0]), dofs_idx_local=[crank_dof])
        scene.step(); step[0] += 1
        render_tick()
    bp = _npy(box.get_pos()); bc = bag_com()
    print(f"  [descend] carrier_z={cur[2]:.4f}  bag_com=({bc[0]:.3f},{bc[1]:.3f},{bc[2]:.3f})  box_z={bp[2]:.4f}")

    # ── Phase 5: crank warmup (0 → -π/2 PD position 램프) ──
    print(f"[phase] 5/7 crank warmup (0.5 s) — 0 → {CRANK_START_Q:+.3f} rad")
    for k in range(N_WARMUP):
        s = (k + 1) / N_WARMUP
        q_target = CRANK_START_Q * s
        crusher.control_dofs_position(np.array([q_target]), dofs_idx_local=[crank_dof])
        crusher.control_dofs_position(np.array([WALL_OFFSET]), dofs_idx_local=[wall_dof])
        carrier.set_pos(final_p, zero_velocity=True)
        scene.step(); step[0] += 1
        render_tick()
    q_now = _npy(crusher.get_dofs_position())[crank_dof]
    print(f"  [warmup] crank actual={q_now:+.3f} rad")

    # ── Phase 6: clamp (Motor2 Left_Wall 닫힘) ──
    print(f"[phase] 6/7 clamp (1.0 s) — Motor2 @ +{WALL_VEL*1000:.0f} mm/s")
    for _ in range(N_CLAMP):
        # 크랭크는 -π/2 유지 (warmup 끝 자세 그대로)
        crusher.control_dofs_position(np.array([CRANK_START_Q]), dofs_idx_local=[crank_dof])
        crusher.control_dofs_velocity(np.array([WALL_VEL]), dofs_idx_local=[wall_dof])
        carrier.set_pos(final_p, zero_velocity=True)
        scene.step(); step[0] += 1
        render_tick()
    wq = _npy(crusher.get_dofs_position())[wall_dof]
    print(f"  [clamp] wall_pos={wq*1000:+.1f} mm")

    # ── Phase 7: crank (Motor1 등속 회전, Motor2 정지) ──
    print(f"[phase] 7/7 crank (4.0 s) — Motor1 @ {CRANK_OMEGA:.3f} rad/s")
    for k in range(N_CRANK):
        crusher.control_dofs_velocity(np.array([CRANK_OMEGA]), dofs_idx_local=[crank_dof])
        crusher.control_dofs_velocity(np.array([0.0]), dofs_idx_local=[wall_dof])
        carrier.set_pos(final_p, zero_velocity=True)
        scene.step(); step[0] += 1
        render_tick()
        if (k + 1) % 1000 == 0:
            cq = _npy(crusher.get_dofs_position())[crank_dof]
            bp = _npy(box.get_pos())
            print(f"  [crank] step={k+1}  crank={cq:+.3f} rad  box=({bp[0]:.3f},{bp[1]:.3f},{bp[2]:.3f})")

    cam_side.stop_recording(save_to_filename=MP4_SIDE, fps=30)
    cam_top.stop_recording(save_to_filename=MP4_TOP, fps=30)
    print(f"\n[saved] side(x,z) → {MP4_SIDE}")
    print(f"[saved] top (x,y) → {MP4_TOP}")
    print(f"[verify] bag_com final = {bag_com()}")
    print(f"[verify] box final     = {_npy(box.get_pos())}")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=True)

"""
Crusher_Pill.py — Crusher 슬롯에서 Wall_back(=L2_Wall3_1) 바로 앞에 알약(정제) 1개를 두고,
docs/Crusher.md §12(제어 알고리즘 — 타격·후퇴 STRIKE/RETRACT FSM)를 그대로 이식해 구동.

봉투/박스 없이 알약(tablets_stl/tablet_R4.0_AR1.00_CV0.20.stl) 하나만 사용.
Left_Wall(슬라이더) 조인트는 이번엔 아예 움직이지 않는다 — 정적 gap(12mm) 그대로 고정.
알약은 fixed=True 로 scene 에 단단히 고정.

── 메커니즘 변경 배경 (docs/Crusher.md §12, docs/DigitalTwin.md §7-7(3) 확인 후) ──
기존엔 크랭크를 한 방향으로 계속 등속 회전만 시켰는데, 알약이 fixed=True(무한 질량)라
크랭크가 거기 막히면 그대로 영구 고착됐다(4초 내내 특정 각도에 멈춤). 그런데 이건 사실
버그가 아니라 실기와 같은 현상이다 — Crusher.md §12 에 따르면 실제 Crusher 도 정제에
막히면 stall(속도≈0) 이 발생하고, 그러면 반대 방향(RETRACT)으로 빼서 다음 타격을
준비한다. 우리 시뮬은 그 stall 을 감지 못하고 같은 방향으로 계속 명령만 내리니 영원히
막혀 있었던 것. → §12-3 STRIKE/RETRACT 2-state FSM 을 그대로 이식:
  STRIKE(+CRANK_OMEGA)  --stall 감지(각도 변화 거의 없음)--> RETRACT(-CRANK_OMEGA)
  RETRACT               --후퇴각 RETRACT_ANGLE 도달--------> STRIKE
이러면 알약이 고정되어 있어도 크랭크가 주기적으로 타격→후퇴→재타격을 반복한다.

센싱: pill.get_contacts(with_entity=crusher) 로 매 스텝 raw 접촉력만 표시(보정 배율·
peak 추적 없음) — stall 이 감지된 순간(=타격 성립 순간)마다 그때의 raw 힘을 로그로 남긴다.

카메라: Crusher_only.py 의 진짜 정면(frontcams["X"], crusher_front_X.png) 과 동일 상대각.
  이 스크립트는 CRUSHER_EULER=(0,0,90) 으로 기체가 90° 돌아가 있어 그 정면은 world-Y 축
  오프셋(이미지 평면 X-Z) 이 된다 — Crusher_Samplebag.py 의 cam_side 와 동일 설정.

출력:
  Sim_result/Crusher_Pill_<ts>_wallfront.mp4  (접촉력 센싱 오버레이 포함)
"""
import os, sys, shutil, tempfile
import xml.etree.ElementTree as ET
from datetime import datetime
import numpy as np
import trimesh as tm
from PIL import Image, ImageDraw, ImageFont

# Windows cp949 한글 깨짐 방지
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

# ── 시뮬 옵션 ────────────────────────────────────────────────────────────────
#   SUBSTEPS 를 늘려 터널링(빠른 접근 중 겹침을 못 잡고 뚫고 지나가는 현상) 완화 —
#   substep_dt 를 절반(1e-4 → 5e-5)으로 줄여 매 스텝 이동거리를 더 작게 쪼갠다.
DT, SUBSTEPS = 1e-3, 20
RENDER_EVERY = 15

# ── 알약 (정제, tablets_stl 원본 메시 재사용 — FEM 스크립트와 동일 스케일/밀도) ──
#   원본(mm→m, PILL_SCALE=1e-3) 은 지름 8mm·두께 4mm — plate 스윕 경로와 잘 안 겹쳤을
#   가능성. 3배(두께 12mm)·2배(두께 8mm) 모두 정적 구조물과도 겹쳐 크랭크가 못 도는
#   고착 문제 발생(4초 내내 -0.7~-0.85rad 부근에 고정) → 1.5배(두께 6mm)로 더 낮춰서
#   메커니즘이 정상 회전하는 한계선을 찾는다.
PILL_SCALE = 1.5e-3    # mm → m × 1.5 (지름 12mm, 두께 6mm)
PILL_RHO   = 1300.0  # kg/m^3 (FEM/fem_uniaxial_compression.py 와 동일)

# ── Crusher 배치 ────────────────────────────────────────────────────────────
CRUSHER_POS   = (0.55, 0.0, 0.0)
CRUSHER_EULER = (0.0, 0.0, 90.0)

# ── 크랭크 PD 게인 + 동작 파라미터 (Crusher_Samplebag.py 와 동일값) ────────
CRANK_KP, CRANK_KV = 2000.0, 100.0
WALL_KP,  WALL_KV  = 5000.0, 500.0   # wall_dof 는 움직이지 않지만 제자리 고정 PD 로 사용
CRANK_START_Q = -np.pi          # 홈 = -180° (L8 헤드 완전 후퇴)
CRANK_RPM     = 8.0             # docs/Crusher.md §1·§11-1 실기 사양(8 RPM, 20 RPM 미만 권장)
CRANK_OMEGA   = CRANK_RPM * 2.0 * np.pi / 60.0   # 0.8378 rad/s (STRIKE 속도) — 기존 π(30RPM)에서 하향

# ── STRIKE/RETRACT FSM (docs/Crusher.md §12-3/§12-7 이식) ──────────────────
#   STRIKE: +CRANK_OMEGA 로 타격. 최근 STALL_WINDOW 스텝 동안 각도 변화가
#     STALL_ANGLE_EPS 미만이면 "정제에 막혀 stall" 로 보고 RETRACT 전환.
#   RETRACT: -CRANK_OMEGA(역방향)로 RETRACT_ANGLE 만큼 후퇴 후 다시 STRIKE.
STALL_WINDOW     = 200          # 0.2s 윈도우 (samples, per step)
STALL_ANGLE_EPS  = 0.02         # rad (~1.1°) — 이보다 적게 움직이면 stall
RETRACT_ANGLE    = 0.35         # rad (~20°, docs 제안 15~30° 범위)

# ── 충돌(rigid contact) 강성 — 터널링 대응으로 대폭 강화 ───────────────────
#   1) 전역 solver: constraint_timeconst 기본 0.01 → 0.004 → 이번엔 0.001 까지 더 빡빡하게
#      (substep_dt=5e-5 기준 20배 — 안전선인 4×substep_dt=2e-4 보다 충분히 크면서도
#      훨씬 강성이 높은 값).
#   2) iterations 도 100 → 200 으로 늘려 접촉 constraint 수렴을 더 정확하게.
#   3) mesh-mesh 충돌은 SDF 로 근사되는데 기본 sdf_cell_size=5mm 는 알약 두께(4mm) 보다
#      커서 그리드가 알약 형상을 거의 못 담아 충돌 자체가 안 잡힌다 — 이게 실제 원인으로
#      보임. 알약/Impact-plate(L9, 10mm 두께) 양쪽 다 훨씬 촘촘한 SDF 로 재설정.
CONTACT_TIMECONST  = 0.001
CONTACT_ITERATIONS = 200
PILL_SDF_CELL  = 0.0003   # 0.3mm (알약 최소 두께 4mm 대비 13셀)
PLATE_SDF_CELL = 0.001    # 1.0mm (crusher 전체 material — L9 plate 두께 10mm 대비 10셀)

# ── 센싱 오버레이 폰트 ───────────────────────────────────────────────────────
FONT_PATHS = ["C:/Windows/Fonts/arialbd.ttf", "C:/Windows/Fonts/arial.ttf"]

# ── 페이즈 step 수 ──────────────────────────────────────────────────────────
N_SETTLE = 800   # 0.8 s  알약 고정 + 크랭크 홈 램프 (wall 은 처음부터 고정)
N_CRANK  = 16000  # 16.0 s — 8RPM 은 30RPM 대비 ~3.75배 느려 사이클당 시간도 늘어남(여러
                   # 타격 사이클 확보 위해 기존 8s → 16s 로 연장)

# ── 카메라: 벽쪽(Left_Wall 바깥, Wall_back 반대편)에서 gap 정면샷 ──────────
CAM_RES  = (960, 720)
CAM_FOV  = 45
CAM_DIST = 0.30

# ── 정적 벽 mesh (Wall_1 ↔ Left_Wall 사이의 좁은 gap 이 타깃) ─────────────
WALL_BACK_MESH    = "L2_Wall3_1"   # "Wall_back" — 내부(고정) 벽
WALL_LEFT_MESH    = "L2_Left_Wall1_1"   # Left_Wall — 바깥쪽(개폐) 벽, 이 쪽이 "벽 정면"
LEFTWALL_BODY_POS = (-0.017802, 0.286278, 0.016542)
LEFTWALL_GEOM_POS = (-0.286278, -0.016542, 0.017802)

# ── 경로: config.json + paths.py (중앙 해석) — 위치 독립 부트스트랩 ──────────
_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
import paths

OUT_DIR  = paths.SIM_RESULT
PILL_STL = os.path.join(paths.TABLETS_STL, "tablet_R4.0_AR1.00_CV0.20.stl")
_TS      = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4_WALLFRONT = os.path.join(OUT_DIR, f"Crusher_Pill_{_TS}_wallfront.mp4")
CRUSHER_SRC_XML = paths.MJCF_MAIN

# ── Crusher MJCF 패치 옵션 ──────────────────────────────────────────────────
WALL_GEOMS_TO_ENABLE = {"base_link", "L1_Wall1_1", "L1_Wall2_1", "L2_Wall3_1"}
L7_LINK3_COM = "0.006 0 -0.005"


# ─────────────────────────────── helpers ────────────────────────────────────
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


def _rgb_of(r):
    a = r[0] if isinstance(r, (tuple, list)) else r
    a = _npy(a)
    return a[..., :3].astype("uint8")


def _font(sz):
    for p in FONT_PATHS:
        if os.path.exists(p):
            return ImageFont.truetype(p, sz)
    return ImageFont.load_default()


def _overlay(rgb, lines, color=(255, 60, 60)):
    """rgb 프레임 중앙 상단에 센싱 텍스트를 그려 새 배열로 반환(각 줄 수평 중앙정렬)."""
    img = Image.fromarray(rgb)
    draw = ImageDraw.Draw(img)
    f = _font(22)
    line_h = 28
    h_img, w_img = rgb.shape[0], rgb.shape[1]
    y0 = 16  # 상단 여백
    for i, line in enumerate(lines):
        bbox = draw.textbbox((0, 0), line, font=f)
        tw = bbox[2] - bbox[0]
        x = (w_img - tw) / 2
        y = y0 + i * line_h
        draw.text((x, y), line, fill=color, font=f)
    return np.asarray(img)


# 정적 벽 geom 변환: quat=(.5,.5,.5,.5) → 아래 행렬, scale 1e-3
_R_GEOM_HALF = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])


def crusher_mesh_world_aabb(mesh_name, body_pos=(0., 0., 0.), geom_pos=(0., 0., 0.)):
    yaw = np.radians(CRUSHER_EULER[2])
    R_e = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                    [np.sin(yaw),  np.cos(yaw), 0.],
                    [0., 0., 1.]])
    v = tm.load(os.path.join(paths.MJCF_DIR, f"{mesh_name}.stl")).vertices * 0.001
    local      = np.asarray(geom_pos) + v
    in_crusher = np.asarray(body_pos) + (_R_GEOM_HALF @ local.T).T
    w          = np.array(CRUSHER_POS) + (R_e @ in_crusher.T).T
    return w.min(axis=0), w.max(axis=0)


def main(use_viewer: bool = True):
    print("="*60)
    print(f" Crusher_Pill — 끝점 알약 고정 + 클램프 + 크랭크 타격 (viewer={use_viewer})")
    print("="*60)
    crusher_xml = _prepare_crusher_mjcf()
    print(f"[crusher] patched MJCF → {crusher_xml}")

    import genesis as gs
    _backend = gs.metal if sys.platform == "darwin" else gs.cuda
    gs.init(backend=_backend, logging_level="warning")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, -9.81)),
        rigid_options=gs.options.RigidOptions(
            constraint_timeconst=CONTACT_TIMECONST, iterations=CONTACT_ITERATIONS),
        vis_options=gs.options.VisOptions(background_color=(0.93, 0.94, 0.96)),
        show_viewer=use_viewer,
    )

    # 알루미늄 plate 2×2 그리드 (작업면)
    PLATE_PATH = paths.ALUMINUM_PLATE
    for p in [(0.5, -0.5, 0), (0.5, 0.5, 0), (-0.5, -0.5, 0), (-0.5, 0.5, 0)]:
        scene.add_entity(
            gs.morphs.Mesh(file=PLATE_PATH, fixed=True, pos=p),
            material=gs.materials.Rigid(),
            surface=gs.surfaces.Default(color=(0.82, 0.82, 0.85), metallic=0.85, roughness=0.3),
        )
    # Crusher — material 로 SDF 촘촘화 (L9 impact-plate 두께 10mm 대비 기본 5mm 셀은 너무 성김)
    crusher = scene.add_entity(
        gs.morphs.MJCF(file=crusher_xml, pos=CRUSHER_POS, euler=CRUSHER_EULER,
                       decimate=False, convexify=False),
        material=gs.materials.Rigid(sdf_cell_size=PLATE_SDF_CELL),
        surface=gs.surfaces.Default(smooth=False),
    )

    # ── gap 위치 해석 (build 전) ────────────────────────────────────────────
    wb_lo, wb_hi = crusher_mesh_world_aabb(WALL_BACK_MESH)
    wl_lo, wl_hi = crusher_mesh_world_aabb(WALL_LEFT_MESH, LEFTWALL_BODY_POS, LEFTWALL_GEOM_POS)
    gap_lo_x, gap_hi_x = sorted([wb_hi[0], wl_lo[0]])
    gap_cx = (gap_lo_x + gap_hi_x) / 2.0
    gap_width = gap_hi_x - gap_lo_x
    y_lo = max(wb_lo[1], wl_lo[1]); y_hi = min(wb_hi[1], wl_hi[1])
    gap_cy = (y_lo + y_hi) / 2.0
    wall_top_z = max(wb_hi[2], wl_hi[2])
    wall_depth = wall_top_z - wb_lo[2]

    # 끝점(target) — 기하 추정(Wall3+margin, gap 중앙, 포켓 중간깊이) 대신, 별도 낙하 테스트
    # (probe_pill_on_plate.py: 크랭크 q=0 고정 후 알약을 위에서 자유낙하)로 실측한 정지 위치를
    # 그대로 사용한다 — 알약이 실제로 crusher 구조물에 얹혀 정지한 좌표라 실접촉이 보장됨.
    target_x, target_y, target_z = 0.2164, -0.0485, 0.0482
    pill_pos = (target_x, target_y, target_z)
    print(f"\n[gap]  {WALL_BACK_MESH}.+x={wb_hi[0]:.4f}  Left_Wall.-x={wl_lo[0]:.4f}  "
          f"width={gap_width*1000:.2f}mm  wall_top_z={wall_top_z:.4f}  depth={wall_depth*1000:.1f}mm")
    print(f"[pill] target(끝점, 낙하테스트 실측 정지좌표) = "
          f"({target_x:.4f}, {target_y:.4f}, {target_z:.4f})")

    # ── 알약 (Rigid, tablets_stl 원본 메시) ────────────────────────────────
    #   원본 mesh 로컬축(PILL_SCALE 적용 후): X=12mm(지름) Y=6mm(두께) Z=12mm(지름).
    #   euler=(0,0,90) 로 두께축(Y)을 world-X(클램프 방향)에 맞춘다.
    #   sdf_cell_size 를 세밀화 — 기본 5mm 셀은 알약 두께(4mm) 보다 커서 SDF 가 형상을
    #   못 담아 충돌이 아예 안 잡히는 게 실제 원인이었음(별도 낙하 테스트로 검증 완료).
    #   fixed=True 로 scene 내 정적(kinematic) 고정("단단히 고정") — 무한 질량 취급되어
    #   절대 움직이지 않는다. 크랭크가 여기 막히면 STRIKE/RETRACT FSM(아래)이 stall 을
    #   감지해 후퇴시키므로 예전처럼 영구 고착되지 않는다. 센싱은 raw 값 그대로 사용
    #   (보정 배율·peak 추적 없음).
    pill = scene.add_entity(
        material=gs.materials.Rigid(rho=PILL_RHO, sdf_cell_size=PILL_SDF_CELL),
        morph=gs.morphs.Mesh(file=PILL_STL, scale=PILL_SCALE, pos=pill_pos, euler=(0, 0, 90),
                             fixed=True),
        surface=gs.surfaces.Default(color=(0.92, 0.90, 0.80)),
    )

    # "정면" 카메라 — world-Y 오프셋(이미지 평면 X-Z, Crusher_Samplebag.py cam_side 와 동일
    #   설정). 이전 -y 오프셋(0,-CAM_DIST,0) 은 실제로는 "후면"이었음(확인됨) → +y 로 반전.
    slot_center = np.array([target_x, target_y, target_z])
    cam_wallfront = scene.add_camera(
        res=CAM_RES, pos=tuple(slot_center + np.array([0.0, CAM_DIST, 0.0])),
        lookat=tuple(slot_center), up=(0.0, 0.0, 1.0), fov=CAM_FOV, GUI=False)
    print(f"[cam] wallfront(front, x-z plane) pos={slot_center + np.array([0.0, CAM_DIST, 0.0])}  "
          f"lookat={slot_center}  (world +y 에서 -y 로 바라봄)")

    scene.build(n_envs=0)

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
    print(f"[ctrl] wall  DOF #{wall_dof}: kp={WALL_KP}, kv={WALL_KV}  (고정 유지, 스윕 안 함)")

    # Left_Wall(슬라이더) 은 이번엔 움직이지 않는다 — 지금 위치(정적 gap)를 그대로
    # PD 목표로 고정해 물리적으로 흔들리지 않게만 잡아둔다.
    wall_hold_q = _npy(crusher.get_dofs_position())[wall_dof]
    print(f"[init] Left_Wall q={wall_hold_q*1000:+.2f}mm (고정, static gap={gap_width*1000:.1f}mm)")

    step = [0]
    frames = []
    last_sense = [0.0, 0]   # [F_now(raw), n_contact_pairs] — 매 스텝 갱신
    state = ["STRIKE"]
    angle_hist = []          # STRIKE 중 최근 STALL_WINDOW 개 크랭크 각도
    retract_start_q = [0.0]
    n_strikes = [0]

    def _sense_contact():
        """pill-crusher raw(비보정) 접촉력: (현재 스텝 최대 접촉력[N], 접촉쌍 수)."""
        c = pill.get_contacts(with_entity=crusher)
        fa = c.get("force_a")
        if fa is None:
            return 0.0, 0
        fa = _npy(fa)
        if fa.ndim < 2 or fa.shape[0] == 0:
            return 0.0, 0
        mags = np.linalg.norm(fa.reshape(fa.shape[0], -1)[:, :3], axis=-1)
        return float(mags.max()), int(fa.shape[0])

    def sense_every_step():
        f_now, n_c = _sense_contact()
        last_sense[0], last_sense[1] = f_now, n_c

    def capture_tick():
        """RENDER_EVERY 마다: 렌더 + 같은 프레임에 최신 센싱값(raw) 텍스트 오버레이."""
        if step[0] % RENDER_EVERY != 0:
            return
        rgb = _rgb_of(cam_wallfront.render(rgb=True))
        cq = _npy(crusher.get_dofs_position())[crank_dof]
        lines = [
            f"t={step[0]*DT:6.3f}s  crank={np.degrees(cq):+6.1f}deg  [{state[0]}]",
            f"contact_pairs={last_sense[1]}  F_now={last_sense[0]:7.3f} N",
        ]
        frames.append(_overlay(rgb, lines))

    # ── Phase 1: settle (알약은 fixed=True 로 이미 단단히 고정됨, 크랭크 홈 램프, wall 고정) ──
    print(f"\n[phase] 1/2 settle ({N_SETTLE*DT:.1f} s) — 알약 고정 확인 + 크랭크 0 → {CRANK_START_Q:+.3f}rad(-180°) 램프")
    for k in range(N_SETTLE):
        s = (k + 1) / N_SETTLE
        crusher.control_dofs_position(np.array([wall_hold_q]), dofs_idx_local=[wall_dof])
        crusher.control_dofs_position(np.array([CRANK_START_Q * s]), dofs_idx_local=[crank_dof])
        scene.step(); step[0] += 1
        sense_every_step()
        capture_tick()
    print(f"  [settle] pill=({_npy(pill.get_pos())}) (fixed=True, 고정)")

    # ── Phase 2: STRIKE/RETRACT FSM (docs/Crusher.md §12-3/§12-7) ──────────
    #   STRIKE: +CRANK_OMEGA 로 타격, stall(최근 STALL_WINDOW 스텝 각도변화 <
    #     STALL_ANGLE_EPS) 감지되면 RETRACT 전환 + 그 순간 접촉력을 [HIT] 로 로그.
    #   RETRACT: -CRANK_OMEGA 로 RETRACT_ANGLE 만큼 후퇴 후 다시 STRIKE. wall 은 계속 고정.
    print(f"[phase] 2/2 STRIKE/RETRACT FSM ({N_CRANK*DT:.1f}s) — "
          f"STRIKE @ {CRANK_OMEGA:.3f} rad/s, stall 시 RETRACT {RETRACT_ANGLE:.2f}rad")
    for k in range(N_CRANK):
        cq = _npy(crusher.get_dofs_position())[crank_dof]

        if state[0] == "STRIKE":
            crusher.control_dofs_velocity(np.array([CRANK_OMEGA]), dofs_idx_local=[crank_dof])
            angle_hist.append(cq)
            if len(angle_hist) > STALL_WINDOW:
                angle_hist.pop(0)
            if len(angle_hist) == STALL_WINDOW and (max(angle_hist) - min(angle_hist)) < STALL_ANGLE_EPS:
                f_now, n_c = _sense_contact()
                n_strikes[0] += 1
                print(f"  [HIT #{n_strikes[0]}] step={step[0]}  t={step[0]*DT:.3f}s  "
                      f"crank={cq:+.3f}rad  F={f_now:.3f}N  contact_pairs={n_c}  → RETRACT")
                state[0] = "RETRACT"
                retract_start_q[0] = cq
                angle_hist.clear()
        else:  # RETRACT
            crusher.control_dofs_velocity(np.array([-CRANK_OMEGA]), dofs_idx_local=[crank_dof])
            if abs(cq - retract_start_q[0]) >= RETRACT_ANGLE:
                print(f"  [RETRACT done] step={step[0]}  t={step[0]*DT:.3f}s  "
                      f"crank={cq:+.3f}rad  → STRIKE")
                state[0] = "STRIKE"
                angle_hist.clear()

        crusher.control_dofs_position(np.array([wall_hold_q]), dofs_idx_local=[wall_dof])
        scene.step(); step[0] += 1
        sense_every_step()
        capture_tick()

    print(f"  [crank] total strikes(stall 감지 횟수) = {n_strikes[0]}")

    gs.tools.animate(frames, MP4_WALLFRONT, fps=30)
    print(f"\n[saved] wallfront(센싱 오버레이 포함) → {MP4_WALLFRONT}")
    print(f"[verify] pill final = {_npy(pill.get_pos())}  total strikes = {n_strikes[0]}")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=True)

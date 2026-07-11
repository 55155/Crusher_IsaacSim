"""
Crusher_only.py — Crusher_IsaacSim 단독 시각화 + Motor1 크랭크 준정적 회전.

목적:
  Crusher 메커니즘만 띄워서 Motor1(L4_Shaft) 를 실 운전 사양 (8 RPM, 12.5 N·m cap)
  으로 회전시키고, 크랭크-슬라이더 폐쇄 루프와 패시브 조인트 / 슬라이더 stroke 가
  Crusher.md §2-2 의 운동학·반력 식과 일치하는지 검증.

제어 방식 (FEM 스크립트와 동일 패턴, fem_uniaxial_compression.py:230 참조):
  control_dofs_velocity(ω_target)  +  set_dofs_force_range(±τ_max)
    · ω_target  : 8 RPM = 0.838 rad/s  (Crusher.md §1 운전 RPM)
    · τ_max     : 12.5 N·m            (Crusher.md §2-2 / §11-3, BL4281+PG42)
    · 슬라이더 반력 F = τ / (r·sin θ)  → 625 N @ θ=90°, 1300 N peak

핵심 처리:
  · 원본 MJCF (Crusher_IsaacSim_colored.xml) 를 Genesis 1.1.0 호환되도록 런타임 패치
    - <equality><joint name="lock_crank"> (joint2 없는 polycoef) 제거
    - <equality><weld> 의 MuJoCo direct solref/solimp 음수 → Genesis 양수형 강화값으로 교체
    - <geom name="ground"> 제거 (별도 plane 사용)
  · 초기 크랭크각 -π/2 를 한 번에 teleport 하면 weld 가 못 따라와서 진동 → 1000 step 동안
    0 → -π/2 PD position 램프로 warmup. 이후 velocity + torque cap 본 시뮬.
  · decimate=False, convexify=False, smooth=False → MuJoCo 와 동일 faceted 정밀 렌더.

출력 (use_viewer=False 일 때):
  Sim_result/Crusher_only.mp4
"""
import os, sys, shutil, tempfile
import xml.etree.ElementTree as ET
import numpy as np

DT, SUBSTEPS = 5e-4, 10     # substep_dt=5e-5 → min_timeconst=1e-4 (weld 0.2ms 까지 안전)
RENDER_EVERY = 40
N_WARMUP = 1000             # 0.5 s, crank 0 → -π/2 PD position 램프

# ── 준정적 운전 사양 (Crusher.md §1, §2-2, §11) ─────────────────────────────
#   · 실기 운전 RPM : 8 RPM  (BL4281 + PG42 1/212 ~ 1/504, "20 RPM 미만 저속")
#   · 크랭크 토크   : 0.185 N·m × η0.5 × 212 ≈ 19.6 N·m  (PG42 1/212)
#                    Crusher.md §2-2 표는 τ=12.5 N·m 로 슬라이더 실측 625 N 매칭
#   · 슬라이더 실측 반력 : ~1300 N (TDP 근처, sin θ → 0 에서 amplification)
#   → 본 시뮬: velocity 제어 + force_range 클램프 (FEM 스크립트와 동일 패턴)
CRANK_RPM           = 8.0
OMEGA               = CRANK_RPM * 2.0 * np.pi / 60.0   # 0.8378 rad/s
CRANK_TORQUE_LIM    = 12.5                              # N·m, Crusher.md §2-2 기준
N_SPIN              = 20000                             # 10 s ≈ 1.33 회전 @ 8 RPM
CRANK_RADIUS_M      = 0.02                              # 슬라이더 기대 stroke = ±2 cm

# 크랭크 PD 게인 — 준정적 (저속) 운전용
#   · WARMUP(position) : kp 가 0 → -π/2 ramp 추종에 필요
#   · SPIN(velocity)   : kv 만 사용, force_range 로 토크 cap
#     v_err 이 작아도 force_range 한계까지 즉시 saturate 되도록 kv 크게.
CRANK_KP = 2000.0
CRANK_KV = 5000.0

START_Q = -np.pi / 2

CRANK_JOINT  = "L3_Bevel_GearBox_1_L4_Shaft_1"
PASSIVE_JOINTS = ["L5_Link1_1_L6_Link2_1", "L6_Link2_1_L7_Link3_1",
                  "L2_Linear_bush_1_L8_Link3_Shaft_1"]

# Motor2 후면 슬라이드 (L2_Left_Wall1_1, 압착 벽)
WALL_JOINT     = "L1_Guide1_1_L2_Left_Wall1_1"
WALL_VEL       = 0.03      # m/s (3 cm/s)
WALL_KP        = 5000.0
WALL_KV        = 500.0
N_WALL_FWD     = 2000      # 1.0 s 전진 (→ +3cm 이동 기대)
N_WALL_BACK    = 2000      # 1.0 s 후진 (→ 0cm 복귀 기대)

# ── 경로: config.json + paths.py (중앙 해석) — 위치 독립 부트스트랩 ──────────
_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
import paths

SRC_XML = paths.MJCF_MAIN
OUT_DIR = paths.SIM_RESULT
MP4_PATH = os.path.join(OUT_DIR, "Crusher_only.mp4")

CAM_POS    = (0.45, -0.45, 0.40)
CAM_LOOKAT = (-0.10, 0.18, 0.07)
CAM_FOV    = 42

# ── 인쇄용 고해상도 스틸 (환경변수로 켬) ────────────────────────────────────
#   PRINT_STILLS=1 : 시뮬 도중 12장을 고해상도·타이트 프레이밍으로 별도 저장
#   FRAMING_TEST=1 : warmup 만 돌리고 1장만 저장 → fov/프레이밍 빠른 확인용
#   STILL_FOV      : 낮출수록 확대(크러셔가 프레임을 꽉 채움). 원본 fov 42는 40% 만 채움.
PRINT_STILLS = os.environ.get("PRINT_STILLS") == "1"
FRAMING_TEST = os.environ.get("FRAMING_TEST") == "1"
FRONT_STILL  = os.environ.get("FRONT_STILL") == "1"      # warmup 후 정면(-Y/-X) 고해상도 1장씩
STILL_RES    = (2400, 1600)                              # 3:2, A4 가로 300DPI 셀에 충분
STILL_FOV    = float(os.environ.get("STILL_FOV", "22"))  # 프레임 꽉 채우도록 당김
FRONT_FOV    = float(os.environ.get("FRONT_FOV", "24"))
FRONT_DIST   = 0.62
STILL_N      = 12                                        # 컨택트시트 셀 수
HIRES_DIR    = os.path.join(OUT_DIR, "crusher_frames_hires")
FRONT_HIRES_DIR = os.path.join(OUT_DIR, "crusher_front_hires")
FRAMING_PNG  = os.path.join(OUT_DIR, "_framing_test.png")
# 지정 크랭크 각도(deg, 콤마구분)에서 정면 스틸 캡처. 예: "0,15,30,45,60,75,90,120"
CRANK_ANGLES     = os.environ.get("CRANK_ANGLES", "")
CRANK_ANGLES_DIR = os.path.join(OUT_DIR, "crusher_crank_angles")

# ── 슬라이더(impact plate) 포커스 뷰 ────────────────────────────────────────
#   HIDE_LEFT_WALL=1 : L2_Left_Wall1_1(압착 벽) 시각 지오메트리 제거 → 슬라이더 가림 해소
#   SLIDER_VIEW=1    : CRANK_ANGLES 캡처 시 impact plate(L9) 를 꽉 잡는 +X 스틸 추가 저장
HIDE_LEFT_WALL   = os.environ.get("HIDE_LEFT_WALL") == "1"
SLIDER_VIEW      = os.environ.get("SLIDER_VIEW") == "1"
SLIDER_ANGLES_DIR = os.path.join(OUT_DIR, "crusher_slider_angles")
#   impact plate(L9) 를 일정 위치에서 고정 프레이밍 — 런타임에 plate geom 월드좌표로 lookat 세팅.
#   SLIDER_SHIFT_Y : plate 중심 대비 시점 좌우 미세조정 [m] (+ = 화면 오른쪽/+Y)
SLIDER_LOOKAT    = (-0.08, 0.236, 0.05)   # set_pose 전 placeholder
SLIDER_DIST      = float(os.environ.get("SLIDER_DIST", "0.42"))
SLIDER_FOV       = float(os.environ.get("SLIDER_FOV", "22"))     # plate stroke 가 보이도록 살짝 당김
SLIDER_SHIFT_Y   = float(os.environ.get("SLIDER_SHIFT_Y", "-0.02"))
SLIDER_SHIFT_Z   = float(os.environ.get("SLIDER_SHIFT_Z", "0.0"))


WALL_GEOMS_TO_ENABLE = {"L1_Wall1_1", "L1_Wall2_1", "L2_Wall3_1"}

# URDF 변환 오류로 L7_Link3_1 의 CoM (x=0.096) 이 geom bbox 바깥 → 안전한 값으로 교체
L7_LINK3_COM = "0.006 0 -0.005"     # bbox [(-0.015, 0.027), (-0.010, 0.010), (-0.010, 0)] 중심


def patch_mjcf(src, dst,
                eq_solref="0.0002 50",
                eq_solimp="0.999 0.99999 1e-5"):
    """원본 MJCF → Genesis 호환 사본.

    - lock_crank <joint> equality 제거 (joint2 없는 polycoef → 미지원)
    - <weld> 유지 + solref/solimp 를 매우 빡빡한 값으로 교체
        timeconst=0.2ms (substep_dt 5e-5 의 4×, Genesis min 통과)
        dampratio=50 (스프링 매우 단단)
        solimp=(0.999, 0.99999, 1e-5)  최대 impedance
    - <geom name="ground"> 제거
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
            # Left Wall(압착 벽) 시각 지오메트리 제거 — 슬라이더 가림 해소.
            #   body/joint 는 유지(DOF 인덱싱 보존), mesh 가 L2_Left_Wall1_1* 인 geom 만 제거.
            if HIDE_LEFT_WALL and body.get("name") == "L2_Left_Wall1_1":
                for g in list(body.findall("geom")):
                    if (g.get("mesh") or "").startswith("L2_Left_Wall1_1"):
                        body.remove(g)
    tree.write(dst)


def _prepare_patched_mjcf():
    tmp_dir = tempfile.mkdtemp(prefix="crusher_mjcf_")
    src_dir = os.path.dirname(SRC_XML)
    for f in os.listdir(src_dir):
        s = os.path.join(src_dir, f)
        if os.path.isfile(s):
            shutil.copy2(s, os.path.join(tmp_dir, f))
    dst = os.path.join(tmp_dir, "Crusher_genesis.xml")
    patch_mjcf(SRC_XML, dst)
    return dst


def _rgb_of(r):
    a = r[0] if isinstance(r, (tuple, list)) else r
    a = a.cpu().numpy() if hasattr(a, "cpu") else np.asarray(a)
    return a[..., :3].astype("uint8")


def _collision_geoms(entity):
    """entity 내 충돌 활성 (contype != 0 또는 conaffinity != 0) geom 리스트."""
    out = []
    for link in entity.links:
        for g in link.geoms:
            if g.contype == 0 and g.conaffinity == 0:
                continue
            out.append(g)
    return out


def _pose_T(geom):
    """현재 geom 의 world-frame transform 4x4 (numpy)."""
    import genesis.utils.geom as gu
    pos = geom.get_pos(); quat = geom.get_quat()
    pos = pos.cpu().numpy() if hasattr(pos, "cpu") else np.asarray(pos)
    quat = quat.cpu().numpy() if hasattr(quat, "cpu") else np.asarray(quat)
    return np.asarray(gu.trans_quat_to_T(pos, quat))


def _make_collision_overlays(scene, geoms, color=(1.0, 0.25, 0.1, 0.45)):
    """각 충돌 geom 의 raw trimesh 를 반투명 오버레이로 등록.
    Returns (debug_obj 리스트). 매 step update_debug_objects 로 위치 갱신."""
    import trimesh as tm_lib
    objs = []
    for g in geoms:
        m = g.get_trimesh().copy()
        m.visual = tm_lib.visual.ColorVisuals(
            vertex_colors=np.tile(np.array([color]), (len(m.vertices), 1)))
        objs.append(scene.draw_debug_mesh(m, T=_pose_T(g)))
    return objs


def main(use_viewer: bool = True, show_collision: bool = False):
    print("="*60)
    print(f" Crusher_only — Motor1 spin (viewer={use_viewer}, show_collision={show_collision})")
    print("="*60)
    patched = _prepare_patched_mjcf()
    print(f"[mjcf] patched → {patched}")

    import genesis as gs
    gs.init(backend=gs.cuda, logging_level="warning")

    scene_kwargs = dict(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, -9.81)),
        vis_options=gs.options.VisOptions(background_color=(0.92, 0.94, 0.97)),
        show_viewer=use_viewer,
    )
    if use_viewer:
        scene_kwargs["viewer_options"] = gs.options.ViewerOptions(
            camera_pos=CAM_POS, camera_lookat=CAM_LOOKAT, camera_fov=CAM_FOV, max_FPS=60)

    scene = gs.Scene(**scene_kwargs)
    crusher = scene.add_entity(
        gs.morphs.MJCF(file=patched, decimate=False, convexify=False),
        surface=gs.surfaces.Default(smooth=False),
    )
    cam = scene.add_camera(res=(960, 720), pos=CAM_POS, lookat=CAM_LOOKAT, fov=CAM_FOV, GUI=False)
    # 인쇄용 고해상도 스틸 전용 카메라 (프레임을 꽉 채우도록 fov 를 당김)
    stillcam = None
    if PRINT_STILLS or FRAMING_TEST:
        os.makedirs(HIRES_DIR, exist_ok=True)
        stillcam = scene.add_camera(res=STILL_RES, pos=CAM_POS, lookat=CAM_LOOKAT,
                                    fov=STILL_FOV, GUI=False)
        print(f"[stills] hi-res still cam: res={STILL_RES} fov={STILL_FOV}  → {HIRES_DIR}")
    # 정면(elevation) 후보 카메라: -Y 방향과 -X 방향 (수평 eye-level)
    frontcams = {}
    if FRONT_STILL or CRANK_ANGLES:
        lx, ly, lz = CAM_LOOKAT
        frontcams["Y"] = scene.add_camera(res=STILL_RES, pos=(lx, ly - FRONT_DIST, lz),
                                          lookat=CAM_LOOKAT, up=(0, 0, 1), fov=FRONT_FOV, GUI=False)
        frontcams["X"] = scene.add_camera(res=STILL_RES, pos=(lx + FRONT_DIST, ly, lz),
                                          lookat=CAM_LOOKAT, up=(0, 0, 1), fov=FRONT_FOV, GUI=False)
        print(f"[front] elevation cams -Y/-X  res={STILL_RES} fov={FRONT_FOV}")
    # 슬라이더(impact plate) 포커스 +X 카메라 — plate 근방을 고정 프레이밍
    slidercam = None
    if SLIDER_VIEW and CRANK_ANGLES:
        sx, sy, sz = SLIDER_LOOKAT
        slidercam = scene.add_camera(res=STILL_RES, pos=(sx + SLIDER_DIST, sy, sz),
                                     lookat=SLIDER_LOOKAT, up=(0, 0, 1), fov=SLIDER_FOV, GUI=False)
        print(f"[slider] plate-focus +X cam  lookat={SLIDER_LOOKAT} dist={SLIDER_DIST} fov={SLIDER_FOV}")
    scene.build(n_envs=0)

    joints = {j.name: j for j in crusher.joints if j.name}
    print("[joints]")
    for n in [CRANK_JOINT, *PASSIVE_JOINTS, WALL_JOINT]:
        j = joints.get(n)
        idx = getattr(j, "dofs_idx_local", None) if j else None
        print(f"   {n:50s} -> dofs_idx_local={idx}")

    def _scalar_dof(name):
        d = joints[name].dofs_idx_local
        return d[0] if isinstance(d, (list, tuple, np.ndarray)) else d

    crank_dof = _scalar_dof(CRANK_JOINT)
    wall_dof  = _scalar_dof(WALL_JOINT)
    passive_dofs = [_scalar_dof(n) for n in PASSIVE_JOINTS]

    # PD 게인 — 크랭크 / 벽 슬라이드 각각
    crusher.set_dofs_kp(np.array([CRANK_KP]), dofs_idx_local=[crank_dof])
    crusher.set_dofs_kv(np.array([CRANK_KV]), dofs_idx_local=[crank_dof])
    crusher.set_dofs_kp(np.array([WALL_KP]),  dofs_idx_local=[wall_dof])
    crusher.set_dofs_kv(np.array([WALL_KV]),  dofs_idx_local=[wall_dof])
    print(f"[ctrl] crank DOF #{crank_dof}: kp={CRANK_KP}, kv={CRANK_KV}")
    print(f"[ctrl] wall  DOF #{wall_dof}: kp={WALL_KP}, kv={WALL_KV}")

    # ── 토크 클램프 (Crusher.md §11-3) ────────────────────────────────────────
    #   실 모터: BL4281 0.185 N·m × η0.5 × 212(reducer) ≈ 19.6 N·m
    #   준정적 운전점은 12.5 N·m (슬라이더 625 N @ θ=90°, 1300 N @ TDP 근처)
    #   force_range 로 PD 토크를 ±CRANK_TORQUE_LIM 으로 clip → 실제 모터 한계 모사
    n_dof = crusher.n_dofs
    fmin = np.full(n_dof, -np.inf)
    fmax = np.full(n_dof,  np.inf)
    fmin[crank_dof] = -CRANK_TORQUE_LIM
    fmax[crank_dof] =  CRANK_TORQUE_LIM
    crusher.set_dofs_force_range(lower=fmin, upper=fmax)
    print(f"[ctrl] crank torque clip: ±{CRANK_TORQUE_LIM:.2f} N·m  "
          f"(준정적 8 RPM, slider F ≈ τ/(r·sin θ) → 625 N @ θ=90°)")

    # 충돌 mesh 오버레이 (viewer + show_collision 일 때만)
    coll_geoms, coll_objs = [], []
    if use_viewer and show_collision:
        coll_geoms = _collision_geoms(crusher)
        coll_objs = _make_collision_overlays(scene, coll_geoms)
        print(f"[collision] overlay {len(coll_geoms)} geom (contype/conaffinity ≠ 0)")
        for g in coll_geoms:
            ln = g.link.name if hasattr(g, "link") else "?"
            print(f"   geom_idx={g.idx}  link={ln:25s}  contype={g.contype}  conaffinity={g.conaffinity}")

    def _refresh_collision():
        if coll_objs:
            Ts = [_pose_T(g) for g in coll_geoms]
            scene.update_debug_objects(coll_objs, Ts)

    cam.start_recording()

    # ── 인쇄용 스틸 캡처 스케줄 (warmup 이후 전 구간에 균등 12장) ───────────────
    #   FRONT_STILL + PRINT_STILLS : 정면(+X) 카메라로 12장 (회전하는 정면 컨택트시트)
    #   PRINT_STILLS 단독           : 아이소메트릭 stillcam 으로 12장
    from PIL import Image as _PILImage
    _gstep = {"n": 0, "saved": 0}
    _total_steps = N_WARMUP + N_SPIN + N_WALL_FWD + N_WALL_BACK
    _capture_set = set()
    if FRONT_STILL and PRINT_STILLS and frontcams:
        _sheet_cam, _sheet_dir, _sheet_pfx = frontcams["X"], FRONT_HIRES_DIR, "crusher_front"
    else:
        _sheet_cam, _sheet_dir, _sheet_pfx = stillcam, HIRES_DIR, "crusher_hires"
    if _sheet_cam is not None and PRINT_STILLS:
        os.makedirs(_sheet_dir, exist_ok=True)
        _capture_set = {int(round(x)) for x in
                        np.linspace(N_WARMUP + 500, _total_steps, STILL_N)}
        print(f"[stills] cam={_sheet_pfx} capture at global steps: {sorted(_capture_set)}")

    def _tick_still():
        """매 scene.step() 뒤 호출 — 전역 step 카운트 + 예약된 스텝에서 스틸 저장."""
        _gstep["n"] += 1
        if _sheet_cam is not None and PRINT_STILLS and _gstep["n"] in _capture_set:
            rgb = _sheet_cam.render(rgb=True)[0]
            _gstep["saved"] += 1
            _PILImage.fromarray(np.asarray(rgb)[..., :3].astype("uint8")).save(
                os.path.join(_sheet_dir, f"{_sheet_pfx}_{_gstep['saved']:02d}.png"))
            print(f"[stills] saved {_gstep['saved']}/{STILL_N} @ step {_gstep['n']}")

    # ── CRANK_ANGLES : 지정 각도들에서 정면(+X) 스틸 캡처하고 종료 ──────────────
    #   0 → max(angle) 까지 PD position 으로 부드럽게 회전(텔레포트 X)하며, 크랭크가
    #   각 목표 각도를 지날 때 정면 스틸 1장씩 저장. (도 단위, 양수 방향 회전)
    if CRANK_ANGLES and frontcams:
        os.makedirs(CRANK_ANGLES_DIR, exist_ok=True)
        angs = [float(a) for a in CRANK_ANGLES.split(",")]
        angs_rad = [np.radians(a) for a in angs]
        order = sorted(range(len(angs)), key=lambda i: angs_rad[i])   # 오름차순으로 캡처
        max_rad = max(angs_rad)
        #  크랭크 kv 가 커서 position 램프론 느림 → velocity 제어로 확실히 회전(spin 과 동일)
        OMEGA_CAP = np.radians(float(os.environ.get("CRANK_DEG_PER_S", "45")))
        N_MAX = int(max_rad / OMEGA_CAP / DT) + 4000
        print(f"[crank] velocity sweep 0 → {max(angs):.0f}° @ {np.degrees(OMEGA_CAP):.0f}°/s, "
              f"capture at {angs}°")
        done = [False] * len(angs)

        if slidercam is not None:
            os.makedirs(SLIDER_ANGLES_DIR, exist_ok=True)
            # impact plate(L9) 월드좌표로 카메라 고정 조준 (slider 카리어가 아닌 plate 중심).
            pp = None
            for link in crusher.links:
                if "L8_Link3_Shaft" in (link.name or ""):
                    for g in link.geoms:
                        if not (g.contype == 0 and g.conaffinity == 0):   # L9 plate = 충돌 활성
                            q = g.get_pos()
                            pp = (q.cpu().numpy() if hasattr(q, "cpu") else np.asarray(q)).reshape(-1)[:3]
                            break
            if pp is not None:
                la = (float(pp[0]), float(pp[1]) + SLIDER_SHIFT_Y, float(pp[2]) + SLIDER_SHIFT_Z)
                slidercam.set_pose(pos=(la[0] + SLIDER_DIST, la[1], la[2]), lookat=la, up=(0, 0, 1))
                print(f"[slider] plate world pos={np.round(pp,4)}  -> lookat={np.round(la,4)}")

        def _save_angle(i):
            deg = int(round(angs[i]))
            rgb = frontcams["X"].render(rgb=True)[0]
            _PILImage.fromarray(np.asarray(rgb)[..., :3].astype("uint8")).save(
                os.path.join(CRANK_ANGLES_DIR, f"crank_{deg:03d}deg.png"))
            if slidercam is not None:                 # 슬라이더(impact plate) 포커스 뷰
                srgb = slidercam.render(rgb=True)[0]
                _PILImage.fromarray(np.asarray(srgb)[..., :3].astype("uint8")).save(
                    os.path.join(SLIDER_ANGLES_DIR, f"slider_{deg:03d}deg.png"))
            print(f"[crank] saved {angs[i]:.0f}°  (q={np.degrees(_qnow()):.1f}°)")
            done[i] = True

        def _qnow():
            return float(crusher.get_dofs_position().cpu().numpy()[crank_dof])

        for i in order:                        # 0° (및 그 이하) 는 시작 자세에서 즉시
            if angs_rad[i] <= 1e-4:
                _save_angle(i)
        for step in range(N_MAX):
            crusher.control_dofs_velocity(np.array([OMEGA_CAP]), dofs_idx_local=[crank_dof])
            scene.step()
            q = _qnow()
            for i in order:
                if not done[i] and q >= angs_rad[i] - 1.5e-3:
                    _save_angle(i)
            if all(done):
                break
        miss = [angs[i] for i in range(len(angs)) if not done[i]]
        if miss:
            for i in range(len(angs)):
                if not done[i]:
                    _save_angle(i)             # 안전망(도달 실패 시 최종 자세)
            print(f"[crank][warn] 도달 못한 각도 최종자세로 저장: {miss}")
        print("[crank] done"); return

    # ── (1) WARMUP : PD position 제어로 0 → -π/2 램프 ──────────────────────────
    #     control_dofs_position 은 다이내믹스 통한 PD 토크 → 텔레포트 없이 부드럽게.
    print(f"[warmup] PD position ramp 0 → {START_Q:+.3f} rad over {N_WARMUP} step")
    for k in range(N_WARMUP):
        target = START_Q * (k + 1) / N_WARMUP
        crusher.control_dofs_position(np.array([target]), dofs_idx_local=[crank_dof])
        scene.step()
        _tick_still()
        if (k + 1) % RENDER_EVERY == 0:
            _refresh_collision()
            cam.render()

    # ── FRAMING_TEST : warmup 자세에서 스틸 1장만 저장하고 종료 (fov 확인용) ────
    if FRAMING_TEST and stillcam is not None:
        rgb = stillcam.render(rgb=True)[0]
        _PILImage.fromarray(np.asarray(rgb)[..., :3].astype("uint8")).save(FRAMING_PNG)
        print(f"[framing] saved {FRAMING_PNG}  res={STILL_RES} fov={STILL_FOV}")
        return

    # ── FRONT_STILL (단독) : warmup 자세에서 정면(-Y/-X) 1장씩 저장하고 종료 ──
    #   PRINT_STILLS 와 함께면 종료하지 않고 전체 회전 → 12장 정면 시트로 진행.
    if FRONT_STILL and frontcams and not PRINT_STILLS:
        for tag, c in frontcams.items():
            rgb = c.render(rgb=True)[0]
            p = os.path.join(OUT_DIR, f"crusher_front_{tag}.png")
            _PILImage.fromarray(np.asarray(rgb)[..., :3].astype("uint8")).save(p)
            print(f"[front] saved {p}  (fov={FRONT_FOV})")
        return

    # ── (2) SPIN : 준정적 등속 회전 (control_dofs_velocity + force_range cap) ──
    #     PD velocity 추종 토크가 ±CRANK_TORQUE_LIM 으로 클램프 → 실 모터 모사.
    #     슬라이더 반력 F_slider = τ_crank / (r·sin θ) (Crusher.md §2-2)
    print(f"[spin]   PD velocity {OMEGA:.4f} rad/s ({CRANK_RPM:.1f} RPM) for {N_SPIN} step "
          f"= {N_SPIN*DT:.1f} s  → ≈ {N_SPIN*DT*OMEGA/(2*np.pi):.2f} rev")
    qlog = []
    l8_dof_idx = passive_dofs[-1]   # L8 slider DOF
    for k in range(N_SPIN):
        t = (k + 1) * DT
        crusher.control_dofs_velocity(np.array([OMEGA]), dofs_idx_local=[crank_dof])
        scene.step()
        _tick_still()
        if (k + 1) % RENDER_EVERY == 0:
            _refresh_collision()
            cam.render()
        if (k + 1) % 200 == 0:
            q   = crusher.get_dofs_position().cpu().numpy()
            tau = crusher.get_dofs_control_force().cpu().numpy()
            theta = q[crank_dof]
            tau_c = float(tau[crank_dof])
            # 슬라이더 등가 반력 (마찰 무시, 준정적): F = τ / (r·|sin θ|)
            s = np.sin(theta)
            f_slider = tau_c / (CRANK_RADIUS_M * s) if abs(s) > 1e-3 else float("nan")
            qlog.append([t, theta, tau_c, q[l8_dof_idx], f_slider,
                         *[q[d] for d in passive_dofs]])
            print(f"  t={t:6.3f}s  θ={theta:+.3f}  τ={tau_c:+6.2f} N·m  "
                  f"slider={q[l8_dof_idx]:+.4f} m  F≈{f_slider:+8.1f} N")

    # ── (3) WALL SLIDE TEST : Motor2 슬라이드 (전진 1s → 후진 1s) ───────────────
    print(f"[wall]   PD velocity test on {WALL_JOINT} (±{WALL_VEL} m/s, 2 s)")
    wlog = []
    # 크랭크는 정지 명령 유지 (체인 잔여 진동 흡수)
    for phase, n, sign in [("fwd", N_WALL_FWD, +1), ("back", N_WALL_BACK, -1)]:
        for k in range(n):
            crusher.control_dofs_velocity(np.array([0.0]),               dofs_idx_local=[crank_dof])
            crusher.control_dofs_velocity(np.array([sign * WALL_VEL]),   dofs_idx_local=[wall_dof])
            scene.step()
            _tick_still()
            if (k + 1) % RENDER_EVERY == 0:
                _refresh_collision(); cam.render()
            if (k + 1) % 200 == 0:
                q = crusher.get_dofs_position().cpu().numpy()
                wpos = float(q[wall_dof])
                wlog.append([phase, (k + 1) * DT, wpos, sign])
                print(f"  wall[{phase}] step={k+1:4d}  pos={wpos:+.4f}  vel_target={sign*WALL_VEL:+.3f}")

    cam.stop_recording(save_to_filename=MP4_PATH, fps=30)
    print(f"[saved] {MP4_PATH}")

    if wlog:
        fwd = np.array([r[2] for r in wlog if r[0] == "fwd"])
        back = np.array([r[2] for r in wlog if r[0] == "back"])
        print(f"[wall] forward  end pos={fwd[-1]:+.4f}  (기대 +{WALL_VEL*N_WALL_FWD*DT:+.4f})")
        print(f"[wall] backward end pos={back[-1]:+.4f}  (기대  {0.0:+.4f})")
        if len(fwd) >= 2:
            v_meas = (fwd[-1] - fwd[0]) / ((len(fwd) - 1) * 200 * DT)
            print(f"[wall] forward  measured vel={v_meas:+.4f} m/s  (target +{WALL_VEL:.4f})")

    if qlog:
        arr = np.array(qlog)
        # 컬럼: 0=t, 1=θ, 2=τ_crank, 3=L8_slider_q, 4=F_slider_est, 5+=passive_qs
        # 정상상태 — warmup + 1 s transient 제외 (8 RPM 에서 1 s ≈ 50°)
        ss_mask = arr[:, 0] > 1.0
        passive_cols = arr[:, 5:]
        spans_full = passive_cols.max(axis=0) - passive_cols.min(axis=0)
        spans_ss   = (passive_cols[ss_mask].max(axis=0) - passive_cols[ss_mask].min(axis=0)
                      if ss_mask.any() else spans_full)
        print("[verify] passive joint Δrange  (full vs steady-state, t>1s):")
        for n, sf, ss in zip(PASSIVE_JOINTS, spans_full, spans_ss):
            print(f"   {n:50s}  full={sf:+.4f}  ss={ss:+.4f}")

        # 토크 / 슬라이더 반력 통계
        tau_ss   = arr[ss_mask, 2] if ss_mask.any() else arr[:, 2]
        f_est_ss = arr[ss_mask, 4] if ss_mask.any() else arr[:, 4]
        f_est_ss = f_est_ss[np.isfinite(f_est_ss)]
        print(f"[torque] crank τ (steady): mean={tau_ss.mean():+.3f}  "
              f"min={tau_ss.min():+.3f}  max={tau_ss.max():+.3f}  "
              f"lim=±{CRANK_TORQUE_LIM:.2f} N·m")
        if f_est_ss.size:
            print(f"[force]  slider F = τ/(r·sin θ) [N]: "
                  f"median={np.median(np.abs(f_est_ss)):.1f}  "
                  f"p95={np.percentile(np.abs(f_est_ss), 95):.1f}  "
                  f"max={np.max(np.abs(f_est_ss)):.1f}  "
                  f"(Crusher.md 실측 ≈ 1300 N peak)")

        # 슬라이더 stroke 검증 — 기대치: ±CRANK_RADIUS_M (=±2cm)
        l8_full = arr[:, 3]
        l8_ss   = arr[ss_mask, 3] if ss_mask.any() else l8_full
        center  = (l8_ss.max() + l8_ss.min()) / 2.0
        stroke_full_p2p = l8_full.max() - l8_full.min()
        stroke_ss_p2p   = l8_ss.max()   - l8_ss.min()
        expected_p2p    = 2 * CRANK_RADIUS_M
        ratio = stroke_ss_p2p / expected_p2p
        print(f"[slider] L8 displacement (m):")
        print(f"   full       : min={l8_full.min():+.4f}  max={l8_full.max():+.4f}  p2p={stroke_full_p2p:.4f}")
        print(f"   steady (t>1s): min={l8_ss.min():+.4f}  max={l8_ss.max():+.4f}  p2p={stroke_ss_p2p:.4f}  center={center:+.4f}")
        print(f"   expected p2p : {expected_p2p:.4f} m (crank radius {CRANK_RADIUS_M*100:.1f} cm × 2)")
        print(f"   ratio measured/expected = {ratio:.2f}  {'✓ OK' if 0.8 <= ratio <= 1.3 else '⚠ check weld stiffness'}")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=True, show_collision=True)

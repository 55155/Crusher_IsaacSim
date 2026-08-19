"""
recovery2_bag_clamp.py — 회수장치2가 샘플백 실링부를 물어 고정하는 **전체 시퀀스**
검증. (2026-08-19, 사용자 지시로 신규 작성 — 새 .py 생성 허락받음)

시퀀스(사용자 제시, 2026-08-19)
------------------------------
  1. 분쇄가 끝난 샘플백을 매니퓰레이터가 **상단 중앙**을 파지한 채 회수장치로 이동
     (상단 중앙 파지가 가장 안정적).
  2. 회수장치의 default 는 **열린 상태 = 샤프트 180도 회전**.
  3. 회수장치 아래 핸들을 고정장치 ServoShaft 가 돌린다 -> 회수장치가 잠긴다.
  4. 잠기면서 양쪽 링크쌍 F_LeftLink-M_LeftLink / F_RightLink-M_RightLink 가 압착.
  5. 샘플백 양옆 실링부가 단단히 고정된다.
  6. 그 후 매니퓰레이터가 파지를 놓는다.
  * 초기에 샘플백 실링부 하단과 F_Top / M_Top 의 위치(z)가 같아야 한다.
  * 슬라이더 안에 **라쳇(RachetGear_1 + PAWL_1)** 이 있어 **역방향으로 돌아가지
    않는다**(사용자 2026-08-19). 이 스크립트의 샤프트 지령은 0 -> 180(열림) ->
    300+deg(잠금) 으로 **단조 증가**라 라쳇과 모순되지 않고, 압착 후에는 그 각도를
    그대로 유지한다(= 라쳇이 풀림을 막는 것과 같은 상태). 즉 별도 라쳇 모델링
    없이도 시퀀스가 성립한다 — 되돌리는 동작이 아예 없기 때문이다.
  * 압착은 "닿기만" 해서는 안 되고 **계속 힘으로 눌러야** 한다(사용자 2026-08-19).
    -> §CLAMP_SLIDER_MM: 링크 틈새를 실링부 자유 두께보다 작게 잡아야 한다.

기구 확인(실측)
--------------
`회전_31`(샤프트) 이 폐루프로 `슬라이더_35`(가동턱)를 구동한다. recovery2_only.py
실측 로그 기준 슬라이더 = 0 -> +35.00mm(q=pi) -> 0 의 왕복이고, **q=pi 에서
+35.00mm 로 최대**다. 즉 사용자가 말한 "180도 = 열림"이 실제 기구와 일치한다.

  슬라이더 s 일 때 (모델 원점 기준, x 단위 mm)
      F_LeftLink_1 : [-60.00, -55.00]   (고정턱, 조인트 없음)
      M_LeftLink_1 : [-55+s, -50+s]     (가동턱, 슬라이더에 실림)
      => F-M 틈새 = s.  s=0 이면 두 링크가 딱 붙고(틈 0), s=35 면 35mm 벌어진다.
      F_Top / M_Top : 둘 다 z[48.00, 58.00] -> **실링부 하단 목표 z = 58.00mm**

빌드 순서 제약(중요)
------------------
IPC 는 초기 배치가 교차하는 것은 물론 **너무 가까운 것도** 거부한다(실측:
봉투-상판 3mm 여유 -> `close_mesh.obj` 저장 후 coupler.build() 사망,
5mm -> 통과). 게다가 scene.build() 시점의 자세는 **qpos0(=턱 닫힘)** 이라,
턱 사이에 봉투를 미리 놓으면 그 자체로 교차한다.
  => 그래서 봉투를 **한참 위(PARK_Z)에 띄운 채로 빌드**하고, 턱을 연 뒤
     `FEMEntity.set_position()` 으로 턱 사이에 투입한다. 투입은 build 이후라
     sanity check 를 타지 않는다.

파지/해제 모델
-------------
매니퓰레이터 파지는 봉투 **상단 중앙** 정점을 hard vertex constraint 로 월드에
고정해 대신한다(로봇 팔은 이 씬에 없다). 압착은 constraint 가 아니라 **IPC 접촉**
으로 일어난다 — 그래야 "링크가 실제로 물어서 잡는가"가 검증된다. 해제는
`remove_vertex_constraints()` 로 전 정점 구속을 한꺼번에 걷어낸다.

  * `set_vertex_constraints` 는 Genesis 1.3.3 에서 IPC 커플러 사용 시 막혀 있다
    ("Vertex constraints are not supported by the IPC coupler."). utills/
    fem_ipc_workarounds.patch_fem_vertex_constraints() 로 그 가드를 연다.
    hard constraint 는 위치를 직접 덮어쓰므로 IPC 여부와 무관하게 성립한다
    (선행 검증: 고정 잔차 0.0000mm).

검증 항목
--------
  A. 턱 열림: 슬라이더가 +35.00mm 에 도달하는가
  B. 투입: 실링부 하단 z 가 58.00mm(F_Top/M_Top 상면)에 맞는가
  C. 압착: 링크쌍이 실링부에 실제로 접촉하는가(틈새 -> 실링부 두께까지 좁힘)
  D. 해제 후 유지: 파지를 놓아도 봉투가 떨어지지 않는가  <- 이 시퀀스의 핵심
  E. IPC 커플링이 살아 있는가(자유 정점이 움직이고 기구물을 뚫지 않는가)

출력: RESULT/recovery2_bag_clamp_<ts>.mp4

사용법(Anaconda Prompt, Windows cmd):
    conda activate crusher_genesis
    cd C:\\Crusher_isaacsim\\Crusher_Genesis\\Recovery2_only
    python recovery2_bag_clamp.py
    python recovery2_bag_clamp.py --viewer
"""
import os
import sys
from datetime import datetime

import numpy as np

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
# utills 는 저장소 루트, config.json 은 Crusher_Genesis/ — 즉 _r 의 부모 쪽이다.
sys.path.insert(0, os.path.join(os.path.dirname(_r), "utills"))
import paths
from fem_ipc_workarounds import patch_fem_vertex_constraints

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
_CLAMP_MM_ENV = os.environ.get("CLAMP_MM", "0.8")   # 파일명 구분용(§CLAMP_SLIDER_MM)
MP4 = os.path.join(OUT_DIR, f"recovery2_bag_clamp_gap{_CLAMP_MM_ENV}mm_{_TS}.mp4")

RECOVERY_MJCF = os.path.join(paths.ROBOTS_DIR, "recovery2_mjcf", "recovery2.xml")
RECOVERY_POS = (0.0, 0.0, 0.0)
# 실링부만 1mm 로 얇게 만든 변형본(2026-08-19, 사용자 요청 "실링부 1mm 정도로").
# 원본 Samplebag_seal_pouch3.stl 은 폭 전체가 균일 6.00mm 라 실링부도 6mm 였다 —
# 열봉합 가장자리로는 비현실적이고 링크가 물어야 할 두께가 너무 컸다. 그래서
# 로컬 |x|>=24mm(가장자리 8mm) 구간의 두께만 6 -> 1mm 로 낮추고 22~24mm 를 전이
# 구간으로 둔 메시를 만들었다(몸통은 6mm 유지, 정점/면 위상 동일 771/1504).
# 기존 thin2mm/thin4mm 변형본은 폭 전체를 균일하게 줄여 몸통까지 납작해지므로 부적합.
# 재생성:
#   V[:,2] *= f,  f = 1.0 (|x|<=22mm) / 1.0->1/6 선형전이 (22~24) / 1/6 (|x|>=24)
BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag", "Samplebag_seal_pouch3_seal1mm.stl")

DT = 5e-3
IPC_D_HAT = 1.0e-4

# 봉투 재질 — 기준은 full_workflow.py(E=4.0e5, t=1.0mm) 의 2026-07-15 2차 튜닝값.
# **E 를 10배 올린 이유(2026-08-19, 사용자 지적 "봉투가 기형적으로 늘어나 있다")**:
# 아래 CLOTH_THICK 을 1.0 -> 0.1mm 로 낮췄는데, 이 값은 접촉 두께이자 동시에
# **막 강성 계수**다(막 강성 ~ E x t). 그대로 두면 400 -> 40 N/m 으로 10배 물러져
# 상단 중앙만 잡고 매달렸을 때 봉투가 12% 늘어나고(높이 90 -> 100mm) 두께가
# 6 -> 22mm 로 벌어졌다. E 를 4.0e5 -> 4.0e6 으로 올려 E x t = 400 N/m 을 복원한다
# — 즉 얇아진 건 접촉 오프셋만이고, 천의 뻣뻣함은 원래대로다.
CLOTH_E, CLOTH_NU, CLOTH_RHO = 4.0e6, 0.499, 200.0
# CLOTH_THICK: full_workflow 는 1.0mm 인데, 실링부 형상이 1mm 인 지금 그 값을 쓰면
# IPC 접촉 오프셋(면당 ~1mm)이 형상 두께를 덮어써 압착이 의미를 잃는다.
# 0.2mm 도 실패했다 — IPC 는 천의 **자기 근접**도 두께의 2배 이상을 요구하는데,
# 실링부를 1/6 로 줄이면서 가장자리 끝단이 0.398mm 까지 좁아져 0.4mm 요구에 걸렸다
# (실측: "cloth_0_0 is too close (distance=0.000398, thickness=0.0004)"). -> 0.1mm.
CLOTH_THICK, CLOTH_BEND = 1.0e-4, 400.0
CLOTH_FRICTION = 0.8
FEM_DAMPING = 0.2

BAG_EULER = (90, 0, 90)        # 로컬 폭축->world Y, 높이축->world Z (full_workflow 와 동일)
BAG_SCALE = 1.0
SEAL_BAND_WIDTH = 0.008        # 1mm 로 얇아진 구간(가장자리 8mm)과 일치시킨다
GRIP_BAND_H = 0.010            # 상단 10mm
GRIP_BAND_W = 0.015            # 중앙 ±15mm  -> "상단 중앙 파지"

SHAFT_JOINT = "회전_31"
SLIDER_JOINT = "슬라이더_35"

# 빌드용 주차 위치: 기구물 최고점(링크 상단 148mm)보다 한참 위 — 교차/근접 모두 회피.
PARK_POS = (-0.035, 0.05920, 0.260)

F_LINK_FRONT_X = -0.055        # F 링크 앞면(가동턱이 다가오는 쪽)
PLATE_TOP_Z = 0.058            # F_Top / M_Top 상면 -> 실링부 하단 목표 z
LINK_MID_Y = 0.05920           # 두 링크 y 중점
# (BAG_PLACE_CLEARANCE 제거 — 정중앙 배치로 바뀌면서 쓰이지 않는다)
# 압착 목표 슬라이더(=F-M 링크 틈새). **실링부 자유 두께보다 작아야 누른다.**
#   경위: 7.0mm -> 실링부 자유 두께 6.31mm 대비 0.68mm 헐거워 압축 0 (닿기만 함).
#         4.0mm -> 압축은 걸릴 값이었지만 유지 구간에서 되풀려 무의미했다(§hold_ratchet).
#   지금은 실링부 형상이 1mm 이므로 0.8mm 로 닫는다. 여기에 IPC 접촉 오프셋
#   (CLOTH_THICK 0.1mm x 2면)이 더해져 실효 두께 약 1.2mm > 틈새 0.8mm 가 되므로
#   약 0.4mm 가 실제로 눌린다.
#   환경변수 CLAMP_MM 으로 덮어쓸 수 있다(값 비교 스윕용):
#       set CLAMP_MM=0.4 && python recovery2_bag_clamp.py
CLAMP_SLIDER_MM = float(_CLAMP_MM_ENV)
# NO_CLOSE=1 : 턱을 닫지 않는 대조군(§4단계). 배치·파지·해제는 동일하게 진행된다.
NO_CLOSE = os.environ.get("NO_CLOSE", "0") == "1"

OPEN_ANGLE = np.pi             # 열림 = 180도
N_OPEN = 700                   # 0 -> pi
# 아래 세 값은 "렌더링 시간이 좀 걸리더라도 천천히"라는 사용자 지시(2026-08-19)에
# 따라 늘렸다. 특히 잠금은 봉투를 밀어 접는 구간이라 급하게 닫으면 천에 충격이
# 들어가 형상이 튄다 — 스텝당 각도 증분을 절반 이하로 줄인다.
N_PLACE = 400                  # 투입 후 파지 상태로 안정화 (200 -> 400)
N_CLOSE = 3500                 # pi -> 압착각 (1500 -> 3500, 슬라이더 감시하며 조기 종료)
N_AFTER_CLAMP = 400            # 완전히 닫힌 상태로 유지 — 이 뒤에야 파지를 놓는다
N_RELEASE = 700                # 파지 해제 후 유지 확인
CLOSE_ANGLE_MAX = 2 * np.pi    # 닫힘은 pi -> 2pi 방향

CAM_POS = (0.075, 0.055, 0.205)
CAM_LOOK = (-0.052, 0.059, 0.095)
CAM_FOV = 46


def main(use_viewer: bool = False):
    print("=" * 70)
    print(f" recovery2_bag_clamp — 실링부 압착 고정 전체 시퀀스 (viewer={use_viewer})")
    print("=" * 70)

    import genesis as gs

    gs.init(backend=gs.gpu, logging_level="warning", precision="32")
    patch_fem_vertex_constraints()      # build 전에 1회

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        rigid_options=gs.options.RigidOptions(
            iterations=200, ls_iterations=150, tolerance=1e-8,
            constraint_timeconst=2 * DT,      # 폐루프 constraint 강화(recovery2_only 와 동일 취지)
        ),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=IPC_D_HAT,
            contact_friction_enable=True,
            two_way_coupling=True,
            # True 면 회수장치2 조립체의 설계상 자기교차(AABB 겹침 57쌍)를 IPC 초기
            # 검사가 잡아 빌드가 죽는다. full_workflow.py / recovery2_only.py 도 False.
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=False,
            constraint_strength_translation=100.0,
            constraint_strength_rotation=100.0,
        ),
        fem_options=gs.options.FEMOptions(damping=FEM_DAMPING),
        vis_options=gs.options.VisOptions(
            background_color=(0.93, 0.94, 0.96),
            ambient_light=(0.3, 0.3, 0.32),
            lights=[
                {"type": "directional", "dir": (-1, -1, -1), "color": (1.0, 1.0, 1.0), "intensity": 6.0},
                {"type": "directional", "dir": (1, 1, -0.6), "color": (1.0, 1.0, 1.0), "intensity": 1.5},
            ],
        ),
        show_viewer=use_viewer,
    )

    # convexify=False: `*_col` 원본 STL 이 열린 채 uipc 로 넘어가면 빌드가 죽는다
    # (compute_mesh_volume 어서션). watertighten 경로로 33개를 닫는다.
    # needs_coup 은 끄지 않는다 — 링크가 봉투를 IPC 로 물어야 하므로.
    # coup_friction: **기본값이 0.1 이다.** 봉투는 friction_mu=0.8 인데 링크 쪽이
    # 0.1 이면 물어도 미끄러진다 — 실측으로 확인한 낙하의 직접 원인이다(정렬을
    # 맞추고 틈새를 실링두께보다 좁혀도 해제 후 11.18mm 떨어졌다). 봉투 쪽과 같은
    # 0.8 로 맞춘다. (실제 봉투-금속 마찰은 0.3~0.6 정도이므로 0.8 은 다소 높은
    # 값이다 — 이 값에서도 못 잡으면 마찰이 아니라 압착력 자체의 문제로 봐야 한다.)
    recovery = scene.add_entity(
        gs.morphs.MJCF(file=RECOVERY_MJCF, pos=RECOVERY_POS, decimate=False, convexify=False),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint", coup_friction=CLOTH_FRICTION),
    )
    bag = scene.add_entity(
        material=gs.materials.FEM.Cloth(
            E=CLOTH_E, nu=CLOTH_NU, rho=CLOTH_RHO,
            thickness=CLOTH_THICK, bending_stiffness=CLOTH_BEND,
            friction_mu=CLOTH_FRICTION,
        ),
        morph=gs.morphs.Mesh(file=BAG_STL, scale=BAG_SCALE, pos=PARK_POS, euler=BAG_EULER),
        surface=gs.surfaces.Default(opacity=0.6, roughness=0.9, double_sided=True),
    )
    cam = scene.add_camera(res=(1280, 960), pos=CAM_POS, lookat=CAM_LOOK, fov=CAM_FOV, GUI=False)

    print("\n[build] scene.build() 시작... (봉투는 PARK_Z 에 띄운 채)")
    scene.build(n_envs=0)
    print(f"[build] 성공  recovery2 n_dofs={recovery.n_dofs}")

    def _npy(x):
        return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)

    shaft_idx = recovery.get_joint(SHAFT_JOINT).dofs_idx_local
    slide_idx = recovery.get_joint(SLIDER_JOINT).dofs_idx_local
    left_link = recovery.get_link("F_LeftLink_1")
    right_link = recovery.get_link("F_RightLink_1")

    def slider_mm():
        return float(_npy(recovery.get_dofs_position(dofs_idx_local=slide_idx)).squeeze()) * 1e3

    def bagp():
        return _npy(bag.get_state().pos).squeeze()

    cam.start_recording(save_to_filename=MP4, fps=30)

    # ── 1. 턱 열기: 샤프트 0 -> 180도 ────────────────────────────────────────
    print(f"\n[1/5 열기] {SHAFT_JOINT} 0 -> 180deg ({N_OPEN}스텝) — 슬라이더가 +35mm 로 벌어져야 한다")
    for k in range(N_OPEN):
        q = OPEN_ANGLE * (k + 1) / N_OPEN
        recovery.set_dofs_position(np.array([q]), dofs_idx_local=shaft_idx)
        scene.step()
        if k % 200 == 0:
            print(f"    k={k:4d} q={np.rad2deg(q):6.1f}deg  슬라이더={slider_mm():+6.2f}mm")
    open_s = slider_mm()
    print(f"[1/5 열기] 완료 — 슬라이더={open_s:+.2f}mm  (F-M 틈새 = 이 값)")

    # ── 2. 봉투 투입: 실링부 하단 z = F_Top/M_Top 상면(58mm) ─────────────────
    p = bagp()
    cur_lo, cur_hi = p.min(axis=0), p.max(axis=0)
    bag_thick = cur_hi[0] - cur_lo[0]
    # ── 투입 위치(2026-08-19 재수정, 사용자 지적) ───────────────────────────
    # **직전 버전은 x 를 "열린 턱 틈새의 정중앙"(-37.5mm)으로 잡았는데 틀렸다.**
    # 열린 상태에서 F_Top 은 x[-110.15,-55.00], M_Top 은 x[-20,+35] 이라 그 중앙은
    # **두 상판 사이 허공**이다 — 봉투 밑에 받쳐줄 게 아무것도 없다. 그래서 봉투가
    # 매달린 채 처졌고(실링부 하단 58.00 -> 47.06mm) 일부 파티클이 늘어나 보였다.
    # 사용자 지적대로 봉투는 **고정 상판 F_Top 쪽**에 있어야 한다(가동 상판 M_Top
    # 위에 놓이면 턱이 움직일 때 같이 끌려간다).
    #
    # 그래서 실링부가 처음부터 **고정턱 앞면 바로 앞**에 오도록 맞춘다:
    #   실링부 x = [-55.00+eps, -55.00+eps+실링두께]  (eps=0.1mm)
    # 이러면 몸통(6mm, 두 링크 사이 55mm 틈 구간에만 존재)이 F_Top 위로 걸쳐
    # 서고, 턱이 닫힐 때 가동턱이 실링부를 밀어야 하는 거리도 최소가 된다.
    # y 는 두 링크 중점, z 는 실링부 하단 = 상판 상면(58mm) 그대로.
    by_park = p[:, 1]
    seal_idx0 = np.concatenate([
        np.where(by_park < by_park.min() + SEAL_BAND_WIDTH)[0],
        np.where(by_park > by_park.max() - SEAL_BAND_WIDTH)[0],
    ])
    sx0 = p[seal_idx0, 0]
    seal_thick0 = sx0.max() - sx0.min()
    seal_cx0 = (sx0.min() + sx0.max()) / 2.0
    # **실링부 중심을 "최종 틈새"의 중심에 맞춘다**(2026-08-19 재수정).
    # 직전 시도는 고정턱 앞면에서 0.1mm 앞에 실링부 뒷면을 붙였는데, 그러면
    # 실링부(1.00mm)가 최종 틈새(0.77mm)보다 앞으로 삐져나온다 — 실측:
    #   틈새 [-55.00,-54.23] vs 실링부 [-54.90,-53.90]
    #   -> 틈새에 든 건 0.67mm 뿐이고 0.33mm 는 가동턱 **앞**에 남는다.
    # 즉 가동턱 면이 실링부 한가운데를 찍어누르는 꼴이라, 구속을 풀면 실링부가
    # 앞으로 밀려 빠지고 봉투가 그대로 주저앉았다(해제 후 낙하 14.37mm).
    # 최종 틈새는 [F앞면, F앞면+CLAMP_SLIDER_MM] 이므로 그 중심에 실링부 중심을
    # 두면 양쪽 링크가 (실링두께-틈새)/2 씩 **대칭으로** 물어 눌러준다.
    tgt_seal_cx = F_LINK_FRONT_X + (CLAMP_SLIDER_MM * 1e-3) / 2.0
    delta = np.array([
        tgt_seal_cx - seal_cx0,                            # x: 실링부를 고정턱 앞면에 붙임
        LINK_MID_Y - (cur_lo[1] + cur_hi[1]) / 2.0,        # y: 두 링크 중점(정중앙)
        PLATE_TOP_Z - cur_lo[2],                           # z: 실링부 하단 = 상판 상면
    ])
    print(f"\n[2/5 투입] 실링부 두께 {seal_thick0*1e3:.2f}mm -> 실링부를 고정턱 앞면"
          f"({F_LINK_FRONT_X*1e3:.2f}mm) 기준 최종 틈새 중심 {tgt_seal_cx*1e3:.2f}mm 에 정렬 (F_Top 쪽)")
    bag.set_position(p + delta)

    # ── 3. 파지: **닫힐 때까지 전 정점을 구속한다**(2026-08-19, 사용자 지시) ──
    # 이전에는 상단 중앙 42정점(5.4%)만 잡았더니 나머지가 처지고 벌어졌다.
    # full_workflow.py 는 같은 문제를 바닥+양측면(약 39%)을 잡아 해결했는데,
    # 사용자 지시는 더 단순하다 — "닫히기 전까지 모든 곳을 다 constraint 걸고,
    # 닫히고 나서 다 푼다". 실제 시퀀스와도 맞다(로봇이 잡고 있는 동안 봉투는
    # 형상을 유지하고, 턱이 완전히 다 닫힌 뒤에야 손을 뗀다).
    # 전 정점을 hard 로 잡으면 봉투는 투입 형상 그대로 정지해 있고, 늘어남/처짐이
    # 원천적으로 발생하지 않는다. 압착은 해제 직후 IPC 가 겹침을 풀며 만들어낸다.
    p_placed = bagp()
    by0, bz0 = p_placed[:, 1], p_placed[:, 2]
    yc0 = (by0.min() + by0.max()) / 2.0
    grip_idx = np.where((bz0 > bz0.max() - GRIP_BAND_H) & (np.abs(by0 - yc0) < GRIP_BAND_W))[0]
    all_idx = np.arange(len(p_placed))
    bag.set_vertex_constraints(verts_idx_local=all_idx.tolist(), is_soft_constraint=False)
    placed_bottom_z = bz0.min()
    print(f"[2/5 투입] 봉투 두께={bag_thick*1e3:.2f}mm — 투입 직후 실링부 하단 "
          f"z={placed_bottom_z*1e3:.2f}mm (목표 {PLATE_TOP_Z*1e3:.2f}mm)")
    print(f"[3/5 파지] **전 정점 {len(all_idx)}개 고정** — 턱이 완전히 닫힐 때까지 유지"
          f" (상단 중앙 파지 대역은 그중 {len(grip_idx)}개)")

    for _ in range(N_PLACE):
        scene.step()
    p = bagp()
    lo, hi = p.min(axis=0), p.max(axis=0)
    print(f"[2/5 투입] 안정화 {N_PLACE}스텝 후:")
    print(f"    x[{lo[0]*1e3:7.2f},{hi[0]*1e3:7.2f}]  y[{lo[1]*1e3:7.2f},{hi[1]*1e3:7.2f}]  "
          f"z[{lo[2]*1e3:7.2f},{hi[2]*1e3:7.2f}]")
    print(f"    실링부 하단 z={lo[2]*1e3:.2f}mm  (목표 {PLATE_TOP_Z*1e3:.2f}mm = F_Top/M_Top 상면)")

    # 정점 그룹: 실링부(좌/우 가장자리) / 파지(상단 중앙) / 자유
    by, bz = p[:, 1], p[:, 2]
    left_idx = np.where(by < by.min() + SEAL_BAND_WIDTH)[0]
    right_idx = np.where(by > by.max() - SEAL_BAND_WIDTH)[0]
    seal_idx = np.concatenate([left_idx, right_idx])
    print(f"    정점 {len(by)}개 — 실링부 좌{len(left_idx)}/우{len(right_idx)}, 파지(상단중앙) {len(grip_idx)}")

    # 압착 목표 슬라이더 = **실링부** 앞면이 F 링크 앞면에서 떨어진 거리.
    # 봉투 전체(hi[0])로 잡으면 안 된다 — 상단 중앙만 파지한 채 매달리면 몸통이
    # 파우치처럼 벌어져(실측 6.01 -> 21.52mm) 가동턱이 '벌어진 몸통 바깥면'에
    # 먼저 닿아 슬라이더 22.5mm 에서 멈춰버린다. 그러면 정작 실링부는 물리지
    # 않는다(1차 실행의 실패 원인). 링크가 실제로 무는 대상은 y 가장자리의
    # 실링부이고 그쪽은 평평하게 남으므로, 실링부 정점의 x 범위로 목표를 잡는다.
    seal_p = p[seal_idx]
    seal_x_lo, seal_x_hi = seal_p[:, 0].min(), seal_p[:, 0].max()
    seal_thick_mm = (seal_x_hi - seal_x_lo) * 1e3
    print(f"    실링부 x[{seal_x_lo*1e3:7.2f},{seal_x_hi*1e3:7.2f}] "
          f"-> 실링부 두께 {seal_thick_mm:.2f}mm "
          f"(몸통 전체 두께 {(hi[0]-lo[0])*1e3:.2f}mm)")
    # **첫 접촉에서 멈추면 안 된다.** 2·3차 실행에서 봉투 현재 형상(또는 실링부
    # 현재 위치)으로 목표를 잡아봤는데 둘 다 22.5mm 가 나왔다 — 상단 중앙만 잡고
    # 매달리면 몸통이 파우치처럼 벌어지고(6.01 -> 21.52mm) 실링부까지 +15mm 앞으로
    # 밀려나서, 가동턱이 '벌어진 바깥면'에 닿자마자 멈춰버린 것이다. 실제 클램프는
    # 거기서 더 닫아 봉투를 납작하게 눌러 실링부를 무는 동작이므로, 목표를 현재
    # 형상이 아니라 **실링부 두께 기준 고정값**으로 준다(그 사이 압축은 IPC 가 한다).
    clamp_s_mm = CLAMP_SLIDER_MM
    print(f"    => 압착 목표 슬라이더 = {clamp_s_mm:.2f}mm (고정값, 실링부 두께 기준)")

    grip_p0 = bagp()[grip_idx].copy()

    # ── 4. 잠금: 샤프트 180 -> 압착각. 슬라이더가 목표에 닿으면 조기 종료 ────
    print(f"\n[4/5 잠금] 샤프트 180deg -> 압착 ({N_CLOSE}스텝 상한, 슬라이더 {clamp_s_mm:.2f}mm 에서 정지)")
    clamped_at = None
    # NO_CLOSE=1 : **대조군**. 배치/파지/해제 타이밍을 전부 동일하게 두고 턱만
    # 닫지 않는다. 이걸 돌려야 "해제 후 낙하량"이 압착 덕분인지 아닌지 판정된다 —
    # 닫았을 때와 낙하가 같다면 압착은 아무 기여도 하지 않는 것이다.
    if NO_CLOSE:
        print("    [대조군] NO_CLOSE=1 — 턱을 닫지 않고 열린 채로 유지한다")
        for _ in range(200):
            recovery.set_dofs_position(np.array([OPEN_ANGLE]), dofs_idx_local=shaft_idx)
            scene.step()
    for k in range(0 if NO_CLOSE else N_CLOSE):
        q = OPEN_ANGLE + (CLOSE_ANGLE_MAX - OPEN_ANGLE) * (k + 1) / N_CLOSE
        recovery.set_dofs_position(np.array([q]), dofs_idx_local=shaft_idx)
        scene.step()
        s = slider_mm()
        if k % 100 == 0:
            # 가동턱이 봉투를 실제로 밀고 있는지 직접 본다. M 링크 x = [-55+s, -50+s]
            # 이므로 이 구간이 실링부 x 와 겹치는 동안 실링부가 -x 로 밀려야 정상이다.
            bp = bagp()
            sx = bp[seal_idx][:, 0]
            print(f"    k={k:4d} q={np.rad2deg(q):6.1f}deg  슬라이더={s:+6.2f}mm  "
                  f"M링크x[{-55+s:7.2f},{-50+s:7.2f}]  실링부x[{sx.min()*1e3:7.2f},{sx.max()*1e3:7.2f}]")
        if s <= clamp_s_mm:
            clamped_at = (k, q, s)
            print(f"    >> 압착 도달: k={k} q={np.rad2deg(q):.1f}deg 슬라이더={s:+.2f}mm")
            break
    if clamped_at is None:
        print(f"    !! 상한까지 돌렸는데 슬라이더가 {slider_mm():+.2f}mm — 목표 미달")

    # ── 라쳇: 압착각을 계속 물고 있어야 한다 ────────────────────────────────
    # **이전 실행들의 실패 원인이 여기였다.** 목표 슬라이더에 도달하면 루프를
    # break 했는데, 그 뒤 유지/해제 구간에서 set_dofs_position 을 다시 주지 않아
    # 폐루프 기구가 그대로 되풀렸다 — 실측: 압착 시점 슬라이더 +4.00mm 가 200스텝
    # 뒤 +31.82mm 로 튕겨나갔다(즉 "물었다가 놓아버림"). 그래서 봉투를 누르는
    # 힘이 0 이었던 것(사용자 지적 "계속 힘으로 누르고 있어야 하는데 못 누른다").
    # 실제 장치에서는 슬라이더 안의 라쳇(RachetGear_1 + PAWL_1)이 역회전을 막아
    # 이 각도를 유지한다. 시뮬에서는 매 스텝 같은 각도를 지령해 그 역할을 대신한다.
    q_hold = clamped_at[1] if clamped_at else (OPEN_ANGLE if NO_CLOSE else CLOSE_ANGLE_MAX)

    def hold_ratchet():
        recovery.set_dofs_position(np.array([q_hold]), dofs_idx_local=shaft_idx)

    for _ in range(N_AFTER_CLAMP):
        hold_ratchet()
        scene.step()

    p_clamped = bagp()
    seal_z_clamped = p_clamped[seal_idx][:, 2].mean()
    com_clamped = p_clamped.mean(axis=0)
    sxc = p_clamped[seal_idx][:, 0]
    seal_thick_clamped_mm = (sxc.max() - sxc.min()) * 1e3
    gap_mm = slider_mm()
    compress_mm = seal_thick_mm - seal_thick_clamped_mm
    print(f"[4/5 잠금] 압착 후 — 실링부 평균 z={seal_z_clamped*1e3:.2f}mm  "
          f"봉투 COM z={com_clamped[2]*1e3:.2f}mm")
    print(f"    실링부 x[{sxc.min()*1e3:7.2f},{sxc.max()*1e3:7.2f}]  "
          f"두께 {seal_thick_mm:.2f} -> {seal_thick_clamped_mm:.2f}mm  (압축 {compress_mm:+.2f}mm)")
    print(f"    링크 틈새 {gap_mm:.2f}mm vs 실링부 두께 {seal_thick_clamped_mm:.2f}mm "
          f"-> {'눌리고 있음' if seal_thick_clamped_mm >= gap_mm - 0.05 else f'헐거움 {gap_mm-seal_thick_clamped_mm:.2f}mm — 누르지 못함'}")

    # ── 5. 파지 해제: 링크만으로 버티는지 ───────────────────────────────────
    # 턱이 **완전히 닫히고 유지까지 끝난 뒤**에 전 정점 구속을 한꺼번에 푼다
    # (사용자 지시 2026-08-19: "다 닫히고 나서 풀어도 늦지 않다"). 이 시점부터
    # 봉투를 잡아주는 건 링크 압착(IPC 접촉/마찰)뿐이다.
    bag.remove_vertex_constraints()
    print(f"\n[5/5 해제] 전 정점 구속 해제 — 이제 링크 압착만으로 버텨야 한다 ({N_RELEASE}스텝)")
    for k in range(N_RELEASE):
        hold_ratchet()          # 라쳇이 계속 물고 있다 — 해제 구간에도 유지
        scene.step()
        if k % 150 == 0:
            bp = bagp()
            c = bp.mean(axis=0)
            sx = bp[seal_idx][:, 0]
            print(f"    k={k:4d}  봉투 COM z={c[2]*1e3:7.2f}mm  "
                  f"(해제 시점 대비 {(c[2]-com_clamped[2])*1e3:+6.2f}mm)  "
                  f"슬라이더={slider_mm():5.2f}mm  실링부두께={(sx.max()-sx.min())*1e3:5.2f}mm")

    p_end = bagp()
    com_end = p_end.mean(axis=0)
    drop_mm = (com_clamped[2] - com_end[2]) * 1e3
    grip_res = np.linalg.norm(p_end[grip_idx] - grip_p0, axis=1).max() * 1e3

    print("\n" + "=" * 70)
    print(" 검증 결과")
    print("=" * 70)
    print(f"  A. 턱 열림 (슬라이더 최대)              : {open_s:+.2f} mm")
    print(f"  B. 실링부 하단 z (목표 {PLATE_TOP_Z*1e3:.1f}mm) : 투입직후 {placed_bottom_z*1e3:.2f} mm / 안정화후 {lo[2]*1e3:.2f} mm")
    if clamped_at:
        print(f"  C. 압착 도달 (샤프트각 / 슬라이더)      : {np.rad2deg(clamped_at[1]):.1f}deg / {clamped_at[2]:+.2f} mm")
    else:
        print(f"  C. 압착 도달                            : 실패 (슬라이더 {slider_mm():+.2f}mm)")
    print(f"  D. 해제 후 낙하량 (COM z 하강)          : {drop_mm:+.2f} mm  "
          f"<- 작을수록 물려 있는 것")
    print(f"  E. 상단중앙 정점 이동(해제 후)          : {grip_res:.2f} mm")
    print(f"     봉투 COM: 압착 {com_clamped[2]*1e3:.2f}mm -> 최종 {com_end[2]*1e3:.2f}mm")

    cam.stop_recording()
    print(f"\n[saved] {MP4}")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer="--viewer" in sys.argv)

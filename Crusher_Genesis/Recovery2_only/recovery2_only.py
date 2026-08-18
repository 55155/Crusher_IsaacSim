"""
recovery2_only.py — 회수장치2 단독 실행/시각 확인용.

assets/robots/recovery2_mjcf/recovery2.xml 를 다른 씬 없이 혼자
바닥 위에 놓고, 메인 샤프트 조인트("회전_31")를 0 -> 2π 로 한
바퀴 돌려서 확인한다.

**에셋 교체(2026-08-19)**: fusion2urdf→URDF→MJCF 를 거친
회수장치2_description/회수장치2.xml → fusion2xml(MJCF 직행) 빌드인
recovery2_mjcf/recovery2.xml (대조표는 그 폴더 README.md). 바뀐 것:
  - 조인트 이름이 Fusion 자동생성 한글 그대로다 — 샤프트 회전_31, 크랭크
    회전_29, 슬라이더 슬라이더_35. (구 빌드가 ParentLink_ChildLink 로 재명명한
    건 옛 Genesis 가 한글 조인트명을 못 읽어서였는데, 1.3.3 은 그대로 읽는다.)
  - 폐루프 <connect> 가 **실제 바디끼리**(Crank_1 ↔ M_Top_1) 로 바뀌었다.
    구 빌드는 fusion2urdf 가 중복 export 한 더미 바디 Crank_1_b 를 Crank_1 에
    묶는 형태였고, 그 둘은 기준자세에서 위치·방향이 같아 anchor 를 공유했다 —
    신 빌드의 두 바디는 서로 다른 곳에 있으므로 잔차를 재려면 body2 쪽 anchor 를
    따로 잡아야 한다(§_loop_err).
  - 구 빌드에서 손으로 보정했던 샤프트 힌지축(로컬 원점에서 31.6mm 벗어나 있던
    것)이 신 빌드에서는 바디 원점과 일치한다.

폐루프 constraint residual을 줄이기 위해 RigidOptions의 constraint solver를
강화(iterations/ls_iterations 상향, dt를 줄여 constraint_timeconst 하한을
낮춤)했다.

충돌 설정(신 빌드 기준):
  - 부품 33개 전부 `*_col` 충돌 geom 을 갖는다(구 빌드는 전 부품 시각 전용).
    IPC 로 넘길 때 열린 메시가 생기지 않도록 convexify=False 로 싣는다(§add_entity).
  - coup_type="two_way_soft_constraint" (조인트 위치 지령 가능)

출력: RESULT/recovery2_only_<ts>.mp4
"""
import os, sys
from datetime import datetime
import numpy as np

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
import paths

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"recovery2_only_{_TS}.mp4")

RECOVERY_MJCF = os.path.join(paths.ROBOTS_DIR, "recovery2_mjcf", "recovery2.xml")
# 회수장치2는 로컬 z 최저점이 이미 ~0 (고정장치와 달리 띄울 필요 없음, bbox 실측: z[0, 0.148])
RECOVERY_POS = (0.0, 0.0, 0.001)

DT = 1.0e-3            # constraint_timeconst 하한(2*dt)을 낮춰 폐루프 constraint를 강화
IPC_D_HAT = 1.0e-4
# 영상 길이 = 스텝수 x dt / realtime_factor 다. 프레임은 scene.step() 안에서
# Camera.update_recording() 이 steps_per_frame(=realtime_factor/(fps*dt)) 간격으로
# 잡는다 — 스크립트가 매 스텝 cam.render() 를 불러도 그 그림은 버려진다(중복 렌더).
# 한 바퀴를 10초에 걸쳐 "정말 느리게" 돌린다(사용자 지시 2026-08-19): dt=1e-3 이므로
# 10000스텝 = 10초. 스텝당 각도 증분도 0.40deg -> 0.036deg 로 줄어 키네마틱 구동이
# 폐루프 구속에 주는 충격이 함께 작아진다.
N_SPIN = 10000         # 메인 샤프트 0 -> 2π (10초)
N_HOLD = 1000          # 끝에서 1초 정지

# 크랭크-슬라이더 메커니즘(Shaft/PULLEY/Crank 및 M_Bottom 슬라이더 이동)이 한
# 화면에 크게 잘 보이도록 가깝고 살짝 낮은 대각선 각도(타이트 프레이밍)
CAM_POS = (0.24, -0.22, 0.20)
CAM_LOOK = (-0.05, 0.06, 0.06)
CAM_FOV = 32

MAIN_SHAFT_JOINT = "회전_31"     # 구 빌드의 base_link_Shaft_copy_1
SLIDER_JOINT = "슬라이더_35"       # 구 빌드의 base_link_M_Bottom_1


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" Recovery2 only — 회수장치2 단독 실행 (viewer={use_viewer})")
    print("=" * 60)

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        rigid_options=gs.options.RigidOptions(
            iterations=200,          # 기본 50 -> 200, equality constraint 수렴 강화
            ls_iterations=150,       # 기본 50 -> 150
            tolerance=1e-8,
            constraint_timeconst=2 * DT,  # dt=1e-3 기준 하한(0.002)까지 낮춤 -> 더 뻣뻣한 constraint
        ),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=IPC_D_HAT,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=False,
            constraint_strength_translation=100.0,
            constraint_strength_rotation=100.0,
        ),
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

    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(coup_type="ipc_only"))

    # convexify=False: 신 에셋은 부품마다 `*_col`(원본 STL) 을 달고 있어 링크
    # 33개가 ABD 강체로 IPC 월드에 들어간다. 기본 경로로 두면
    # RightSlider1_1/Shaft_copy_1 의 충돌 메시가 열린 채로 uipc 에 넘어가 빌드가
    # 죽는다(AffineBodyConstitution -> compute_mesh_volume: "Calculating volume
    # of open trimesh is meaningless"). convexify=False 는 Genesis 의
    # watertighten(기본 5) 경로를 타서 33개 전부 닫힌 메시가 된다(실측: 열린
    # geom 0개, IPC 빌드+스텝 통과).
    recovery = scene.add_entity(
        gs.morphs.MJCF(file=RECOVERY_MJCF, pos=RECOVERY_POS, decimate=False, convexify=False),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
    )

    cam = scene.add_camera(res=(1280, 960), pos=CAM_POS, lookat=CAM_LOOK, fov=CAM_FOV, GUI=False)

    print("\n[build] scene.build() 시작...")
    scene.build(n_envs=0)
    print(f"[build] 성공  n_dofs={recovery.n_dofs}")

    import scipy.spatial.transform as T

    dof_idx = recovery.get_joint(MAIN_SHAFT_JOINT).dofs_idx_local
    slide_idx = recovery.get_joint(SLIDER_JOINT).dofs_idx_local
    # 폐루프 <connect name="connect_Crank_1_M_Top_1"> 가 묶는 실제 바디 두 개.
    crank = recovery.get_link("Crank_1")
    mtop = recovery.get_link("M_Top_1")

    def _pose(link):
        p = link.get_pos().cpu().numpy().squeeze()
        q = link.get_quat().cpu().numpy().squeeze()          # (w, x, y, z)
        return p, T.Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()

    # mujoco 의 connect 제약은 "anchor 를 body1 좌표계로 본 점"과 "컴파일 시점
    # (기준자세)에 자동 유도된 같은 월드 점을 body2 좌표계로 본 점"을 일치시킨다.
    # ANCHOR1 은 XML 에 적힌 값(Crank_1 로컬), ANCHOR2 는 빌드 직후 자세에서
    # 그대로 역산한다 — 구 빌드처럼 두 바디가 겹쳐 있지 않으므로 공유할 수 없다.
    ANCHOR1 = np.array([0.06, 0.0, 0.006])
    _p1, _R1 = _pose(crank)
    _p2, _R2 = _pose(mtop)
    ANCHOR2 = _R2.T @ (_p1 + _R1 @ ANCHOR1 - _p2)

    def _loop_err():
        p1, R1 = _pose(crank)
        p2, R2 = _pose(mtop)
        return float(np.linalg.norm((p1 + R1 @ ANCHOR1) - (p2 + R2 @ ANCHOR2)))

    # Genesis 1.3.x: 파일명/fps 가 start_recording 으로 옮겨졌고 stop_recording()
    # 은 인자를 안 받는다(full_workflow.py 와 동일하게 이전).
    cam.start_recording(save_to_filename=MP4, fps=30)

    print(f"\n[spin] {MAIN_SHAFT_JOINT} 0 -> 2π ({N_SPIN}스텝)")
    for k in range(N_SPIN):
        q = 2 * np.pi * (k + 1) / N_SPIN
        recovery.set_dofs_position(np.array([q]), dofs_idx_local=dof_idx)
        scene.step()          # 녹화 프레임은 이 안에서 잡힌다(별도 render 불필요)
        if k % 1000 == 0:
            slide_q = recovery.get_dofs_position(dofs_idx_local=slide_idx).cpu().numpy().squeeze()
            print(f"    k={k:4d} q={q:.3f}rad  loop_err={_loop_err()*1e3:.3f}mm  slider={slide_q*1e3:+.2f}mm")

    print(f"[hold] 정지 유지 ({N_HOLD}스텝)")
    for k in range(N_HOLD):
        scene.step()

    cam.stop_recording()
    print(f"\n[saved] {MP4}")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

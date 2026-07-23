"""
recovery2_only.py — 회수장치2 단독 실행/시각 확인용.

assets/robots/회수장치2_description/회수장치2.xml 를 다른 씬 없이 혼자
바닥 위에 놓고, 메인 샤프트 조인트("base_link_Shaft_copy_1")를 0 -> 2π 로 한
바퀴 돌려서 확인한다.

조인트 이름은 전부 ParentLink_ChildLink 형식으로 재명명됨(한글 조인트명이 깨지는
문제 우회). PULLEY_1측 크랭크(Crank_1_b, 구 "회전 29")와 M_Top_1측 크랭크(Crank_1,
구 "회전 34")는 fusion2urdf가 중복 export한 동일 물리 부품 -- 원래 Fusion CAD의
크랭크-슬라이더 폐루프를 재구성하기 위해 <equality><connect>로 위치 결속(자세한
유도 과정은 회수장치2.xml 주석 참고). base_link_Shaft_copy_1 joint의 축(pos)도
Shaft_copy_1.stl 실측 중심선에 맞춰 보정됨(원래 로컬 원점에서 31.6mm 벗어나
있었음).

폐루프 constraint residual을 줄이기 위해 RigidOptions의 constraint solver를
강화(iterations/ls_iterations 상향, dt를 줄여 constraint_timeconst 하한을
낮춤)했다.

배치/충돌 설정은 Fixture_only/fixture_only.py 패턴과 동일:
  - 전 부품 시각 전용(contype=0/conaffinity=0) — 아직 어느 부위가 실제
    충돌에 관여해야 하는지 정해지지 않아, 고정장치의 안전한 기본값을 따름
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

RECOVERY_MJCF = os.path.join(paths.ROBOTS_DIR, "회수장치2_description", "회수장치2.xml")
# 회수장치2는 로컬 z 최저점이 이미 ~0 (고정장치와 달리 띄울 필요 없음, bbox 실측: z[0, 0.148])
RECOVERY_POS = (0.0, 0.0, 0.001)

DT = 1.0e-3            # constraint_timeconst 하한(2*dt)을 낮춰 폐루프 constraint를 강화
IPC_D_HAT = 1.0e-4
N_SPIN = 900           # 메인 샤프트 0 -> 2π 회전 구간 (dt 축소에 맞춰 스텝수 늘림)
N_HOLD = 90            # 끝에서 잠깐 정지

# 크랭크-슬라이더 메커니즘(Shaft/PULLEY/Crank 및 M_Bottom 슬라이더 이동)이 한
# 화면에 크게 잘 보이도록 가깝고 살짝 낮은 대각선 각도(타이트 프레이밍)
CAM_POS = (0.24, -0.22, 0.20)
CAM_LOOK = (-0.05, 0.06, 0.06)
CAM_FOV = 32

MAIN_SHAFT_JOINT = "base_link_Shaft_copy_1"


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

    recovery = scene.add_entity(
        gs.morphs.MJCF(file=RECOVERY_MJCF, pos=RECOVERY_POS, decimate=False),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
    )

    cam = scene.add_camera(res=(1280, 960), pos=CAM_POS, lookat=CAM_LOOK, fov=CAM_FOV, GUI=False)

    print("\n[build] scene.build() 시작...")
    scene.build(n_envs=0)
    print(f"[build] 성공  n_dofs={recovery.n_dofs}")

    dof_idx = recovery.get_joint(MAIN_SHAFT_JOINT).dofs_idx_local
    crank_pulley = recovery.get_link("Crank_1_b")
    crank_mtop = recovery.get_link("Crank_1")
    anchor_local = np.array([0.06, 0.0, 0.006])
    slide_idx = recovery.get_joint("base_link_M_Bottom_1").dofs_idx_local

    def _loop_err():
        # mujoco의 connect 제약은 "anchor를 body1좌표계로 본 점"과 "(컴파일 시점
        # 기준으로 자동 유도된) 같은 anchor를 body2좌표계로 본 점"을 일치시킨다.
        # Crank_1_b/Crank_1이 기준자세에서 위치+방향이 완전히 같으므로 두 anchor
        # 값이 수치적으로 동일해짐 -- 따라서 "먼 점(far1) vs 상대방 origin(p2)"이
        # 아니라 "far1 vs far2"(둘 다 같은 anchor 적용)를 비교해야 진짜 잔차가 나옴.
        import scipy.spatial.transform as T
        p1 = crank_pulley.get_pos().cpu().numpy().squeeze()
        quat1 = crank_pulley.get_quat().cpu().numpy().squeeze()
        p2 = crank_mtop.get_pos().cpu().numpy().squeeze()
        quat2 = crank_mtop.get_quat().cpu().numpy().squeeze()
        r1 = T.Rotation.from_quat([quat1[1], quat1[2], quat1[3], quat1[0]]).as_matrix()
        r2 = T.Rotation.from_quat([quat2[1], quat2[2], quat2[3], quat2[0]]).as_matrix()
        far1 = p1 + r1 @ anchor_local
        far2 = p2 + r2 @ anchor_local
        return float(np.linalg.norm(far1 - far2))

    cam.start_recording()

    print(f"\n[spin] {MAIN_SHAFT_JOINT} 0 -> 2π ({N_SPIN}스텝)")
    for k in range(N_SPIN):
        q = 2 * np.pi * (k + 1) / N_SPIN
        recovery.set_dofs_position(np.array([q]), dofs_idx_local=dof_idx)
        scene.step()
        cam.render()
        if k % 100 == 0:
            slide_q = recovery.get_dofs_position(dofs_idx_local=slide_idx).cpu().numpy().squeeze()
            print(f"    k={k:4d} q={q:.3f}rad  loop_err={_loop_err()*1e3:.3f}mm  slider={slide_q*1e3:+.2f}mm")

    print(f"[hold] 정지 유지 ({N_HOLD}스텝)")
    for k in range(N_HOLD):
        scene.step()
        cam.render()

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved] {MP4}")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

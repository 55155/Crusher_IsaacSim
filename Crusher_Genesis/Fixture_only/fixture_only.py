"""
fixture_only.py — 고정장치(fixture jig) 단독 실행/시각 확인용.

Crusher_M0609_RG2_Tablet_Samplebag/full_workflow.py 에 심어놓은 고정장치
(assets/robots/고정장치_description/고정장치.xml)를 다른 씬 없이 혼자
바닥에 놓고 Servo3_ServoShaft 힌지(Jig_1이 매달린 유일한 실제 DOF)를
0 -> 2π 로 한 바퀴 돌려서 영상으로 남긴다.

배치/충돌 설정은 full_workflow.py와 동일:
  - base_link/L1/Back/R1/F1/MotorDriver/T1/Servo1~3/ServoShaft: 시각 전용
    (contype=0/conaffinity=0)
  - Jig_1만 CoACD 64-hull로 충돌 활성화
  - coup_type="two_way_soft_constraint" (조인트 위치 지령 가능)

출력: RESULT/fixture_only_<ts>.mp4
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
MP4 = os.path.join(OUT_DIR, f"fixture_only_{_TS}.mp4")

FIXTURE_MJCF = os.path.join(paths.ROBOTS_DIR, "고정장치_description", "고정장치.xml")
# 조립체 로컬 z 최저점(~-0.117m)이 바닥(z=0) 위로 오도록 여유를 둔 배치
# (full_workflow.py와 동일 값) — 여기서는 혼자 있으니 원점(0,0)에 둔다.
FIXTURE_POS = (0.0, 0.0, 0.12)

DT = 5e-3
IPC_D_HAT = 1.0e-4
N_SPIN = 300          # 조인트 0 -> 2π 회전 구간
N_HOLD = 60            # 끝에서 잠깐 정지

CAM_POS = (0.75, -0.75, 0.55)
CAM_LOOK = (0.06, -0.06, 0.05)


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" Fixture only — 고정장치 단독 실행 (viewer={use_viewer})")
    print("=" * 60)

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

    fixture = scene.add_entity(
        gs.morphs.MJCF(file=FIXTURE_MJCF, pos=FIXTURE_POS, decimate=False),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
    )

    cam = scene.add_camera(res=(1280, 960), pos=CAM_POS, lookat=CAM_LOOK, fov=45, GUI=False)

    print("\n[build] scene.build() 시작...")
    scene.build(n_envs=0)
    print(f"[build] 성공  n_dofs={fixture.n_dofs}")

    # Genesis 1.3.x: 파일명/fps 가 start_recording 으로 옮겨졌고 stop_recording()
    # 은 인자를 안 받는다(full_workflow.py 와 동일하게 이전).
    cam.start_recording(save_to_filename=MP4, fps=30)

    print(f"\n[spin] Servo3_ServoShaft 0 -> 2π ({N_SPIN}스텝)")
    for k in range(N_SPIN):
        q = 2 * np.pi * (k + 1) / N_SPIN
        fixture.set_dofs_position(np.array([q]))
        scene.step()
        cam.render()
        if k % 50 == 0:
            print(f"    k={k:4d} q={q:.3f}rad")

    print(f"[hold] 정지 유지 ({N_HOLD}스텝)")
    for k in range(N_HOLD):
        scene.step()
        cam.render()

    cam.stop_recording()
    print(f"\n[saved] {MP4}")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

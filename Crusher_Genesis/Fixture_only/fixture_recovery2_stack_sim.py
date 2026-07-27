"""
fixture_recovery2_stack_sim.py — 고정장치 위에 회수장치2를 얹은 조립 상태에서
Servo3_ServoShaft(Jig_1이 매달린 유일한 회전 DOF)를 한 바퀴(0 -> 2pi) 돌려보는
CLI 시뮬레이션. 사용자가 아나콘다 프롬프트에서 직접 실행해 Jig-회수장치2
오버랩이 실제로 해소됐는지 눈으로 확인하기 위한 용도.

배치(2026-07-27):
  - FIXTURE_POS=(0,0,0.12) — fixture_only.py와 동일(단독 실행 관례).
  - RECOVERY2_POS — 정렬점(고정장치 ServoShaft 중심 vs 회수장치2 자체 정렬점)을
    X,Y,Z 모두 일치시킨 뒤, Jig_1 회전 스윕 시 겹치는 문제(hull scale 누락
    버그를 고치고 나서 발견 — 64개 중 35개 겹침) 때문에 Z만 +25mm 올렸다.
    회전(요, yaw)은 아직 안 맞춰져 있음(사용자가 추후 지시 예정) — 지금은
    오버랩 해소만 확인하는 용도.
  - 회수장치2의 Gear(RachetGear_1)/Crank(Crank_1) 부품은 검정색으로 변경(사용자 지시).
  - T1 충돌은 되돌림(2026-07-27): T1과 Jig_1이 CAD상 원래부터 맞닿아있는 구조라
    (샤프트-브라켓 관계로 추정) 둘 다 collision을 켜면 IPC build 자체가
    "Intersection detected"로 죽는다(8가지 조합 격리 테스트로 T1+Jig_1 조합만
    실패함을 확인, contype/conaffinity 비트마스크로 자기충돌을 걸러도 소용없음 —
    IPC의 초기 유효성 검사는 필터링을 무시하고 순수 메시 교차만 봄). L1/R1은
    Jig_1과 안 겹쳐서 충돌 유지, T1만 시각 전용으로 되돌렸다.

사용법(Anaconda Prompt, Windows cmd):
    conda activate crusher_genesis
    cd C:\\Crusher_isaacsim\\Crusher_Genesis\\Fixture_only

    REM 1) 라이브 뷰어로 직접 돌려보며 확인 (권장 - 마우스로 회전/줌 가능)
    set VIEWER=1 && python fixture_recovery2_stack_sim.py

    REM 2) 헤드리스로 영상만 저장(뷰어 없이)
    python fixture_recovery2_stack_sim.py

매 스텝 Jig_1 hull과 회수장치2 사이 AABB 겹침 개수도 콘솔에 출력한다(0이어야 정상).

출력(헤드리스 모드): RESULT/fixture_recovery2_stack_sim_<ts>.mp4
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
MP4 = os.path.join(OUT_DIR, f"fixture_recovery2_stack_sim_{_TS}.mp4")

FIXTURE_MJCF = os.path.join(paths.ROBOTS_DIR, "고정장치_description", "고정장치.xml")
FIXTURE_POS = (0.0, 0.0, 0.12)

RECOVERY2_MJCF = os.path.join(paths.ROBOTS_DIR, "회수장치2_description", "회수장치2.xml")
RECOVERY2_POS = (-0.063985, 0.01328, 0.294524)  # +25mm 오버랩 해소(full_workflow.py 주석 참고)

DT = 5e-3
IPC_D_HAT = 1.0e-4
N_SPIN = 300   # 조인트 0 -> 2pi 회전 구간
N_HOLD = 60

CAM_POS = (0.55, -0.55, 0.45)
CAM_LOOK = (-0.09, 0.03, 0.2)


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" 고정장치+회수장치2 스택 시뮬레이션 (viewer={use_viewer})")
    print("=" * 60)

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=IPC_D_HAT,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=True,
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
    recovery = scene.add_entity(
        gs.morphs.MJCF(file=RECOVERY2_MJCF, pos=RECOVERY2_POS, decimate=False),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
    )

    cam = None
    if not use_viewer:
        cam = scene.add_camera(res=(1280, 960), pos=CAM_POS, lookat=CAM_LOOK, fov=45, GUI=False)

    print("\n[build] scene.build() 시작...")
    scene.build(n_envs=0)
    print(f"[build] 성공  fixture n_dofs={fixture.n_dofs}  recovery2 n_dofs={recovery.n_dofs}")

    jig_geoms = [g for g in fixture.geoms if g.link.name == "ServoShaft_1" and g.contype == 1]
    rec_aabb = recovery.get_vAABB()
    rec_aabb = rec_aabb.cpu().numpy() if hasattr(rec_aabb, "cpu") else np.asarray(rec_aabb)

    def _npy(x):
        return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)

    def check_overlap():
        n = 0
        for g in jig_geoms:
            bb = _npy(g.get_AABB())
            if all(bb[0][i] <= rec_aabb[1][i] and rec_aabb[0][i] <= bb[1][i] for i in range(3)):
                n += 1
        return n

    if cam is not None:
        cam.start_recording()

    print(f"\n[spin] Servo3_ServoShaft 0 -> 2pi ({N_SPIN}스텝) — Jig-회수장치2 오버랩 실시간 확인")
    for k in range(N_SPIN):
        q = 2 * np.pi * (k + 1) / N_SPIN
        fixture.set_dofs_position(np.array([q]))
        scene.step()
        if cam is not None:
            cam.render()
        if k % 30 == 0:
            n_ov = check_overlap()
            flag = "OK" if n_ov == 0 else f"!! 오버랩 {n_ov}/{len(jig_geoms)}"
            print(f"    k={k:4d} q={q:.3f}rad  {flag}")

    print(f"[hold] 정지 유지 ({N_HOLD}스텝)")
    for k in range(N_HOLD):
        scene.step()
        if cam is not None:
            cam.render()

    if cam is not None:
        cam.stop_recording(save_to_filename=MP4, fps=30)
        print(f"\n[saved] {MP4}")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

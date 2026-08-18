"""
fixture_recovery2_stack_sim.py — 고정장치 위에 회수장치2를 얹은 조립 상태에서
Servo3_ServoShaft(Jig_1이 매달린 유일한 회전 DOF)를 한 바퀴(0 -> 2pi) 돌려보는
CLI 시뮬레이션. 사용자가 아나콘다 프롬프트에서 직접 실행해 Jig-회수장치2
오버랩이 실제로 해소됐는지 눈으로 확인하기 위한 용도.

배치(2026-07-27~29):
  - FIXTURE_POS=(0,0,0.12) — fixture_only.py와 동일(단독 실행 관례).
  - RECOVERY2_POS — 정렬점(고정장치 ServoShaft 중심 vs 회수장치2 자체 정렬점)을
    X,Y,Z 모두 일치시킨 원래 위치. 한때 Jig_1 hull과 회수장치2 "시각적" 형상이
    35/64개 겹쳐서 Z를 +25mm 띄웠었으나, Jig hull 실제 형상을 렌더링해 확인한
    결과 Jig는 담아 고정하는 컵이 아니라 **양쪽 기둥 2개로 회수장치2의 Shaft
    Handle(ShaftHandle_1)을 돌려주는 포크/렌치 구조**였다(사용자 확인) — 즉
    샤프트 쪽과 근접/접촉하는 건 의도된 동력전달이라 인위적 간격을 걷어냈다.
    ShaftHandle_1 자체(실제 충돌 hull 활성화 후)는 이 위치에서 Jig와 전혀 안
    겹친다(0/64, 전 회전각) — 요(yaw)를 아직 안 맞춰서(사용자가 추후 지시
    예정) 기둥이 손잡이를 실제로 미는 각도까지는 아니다.
  - 회수장치2의 Gear(RachetGear_1)/Crank(Crank_1) 부품은 검정색으로 변경(사용자 지시).
  - T1 충돌은 되돌림(2026-07-27): T1과 Jig_1이 CAD상 원래부터 맞닿아있는 구조라
    (샤프트-브라켓 관계로 추정) 둘 다 collision을 켜면 IPC build 자체가
    "Intersection detected"로 죽는다(8가지 조합 격리 테스트로 T1+Jig_1 조합만
    실패함을 확인, contype/conaffinity 비트마스크로 자기충돌을 걸러도 소용없음 —
    IPC의 초기 유효성 검사는 필터링을 무시하고 순수 메시 교차만 봄). L1/R1은
    Jig_1과 안 겹쳐서 충돌 유지, T1만 시각 전용으로 되돌렸다.

**에셋 교체(2026-08-19)**: 회수장치2_description/회수장치2.xml → fusion2xml
(MJCF 직행) 빌드인 recovery2_mjcf/recovery2.xml (대조표는 그 폴더 README.md).
RECOVERY2_POS 는 그대로 둔다 — 정렬점 (-74.015, 59.22, 1.776)mm 는 Fusion 조인트
앵커 값인데 구 빌드는 자기 샤프트 힌지축이 그 앵커에서 (+30.92, -6.62, 0)mm
어긋나 있었고 신 빌드는 0.01mm 안에서 일치한다. 즉 **같은 POS 로 회수장치2
형상이 고정장치 기준 (+30.92, -6.62, 0)mm 옮겨 앉으며, 그래야 원래 의도한
"Jig 포크가 ShaftHandle 을 돌린다"는 정렬이 실제로 성립한다.** 아래 본문의
"ShaftHandle 은 Jig 와 전혀 안 겹친다(0/64, 전 회전각)"는 구 빌드에서 잰 값이라
신 빌드에서는 다시 재야 한다.

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

RECOVERY2_MJCF = os.path.join(paths.ROBOTS_DIR, "recovery2_mjcf", "recovery2.xml")
RECOVERY2_POS = (-0.063985, 0.01328, 0.269524)  # 원래 정렬점 복귀(full_workflow.py 주석 참고)

DT = 5e-3
IPC_D_HAT = 1.0e-4
# 영상 길이 = 스텝수 x dt / realtime_factor (프레임은 scene.step() 안에서
# Camera.update_recording() 이 잡는다 — 매 스텝 cam.render() 를 부르면 그 그림은
# 버려지는 중복 렌더다). 한 바퀴를 10초에 걸쳐 느리게 돌린다(사용자 지시
# 2026-08-19): dt=5e-3 이므로 2000스텝 = 10초.
N_SPIN = 2000  # 조인트 0 -> 2pi 회전 구간 (10초)
N_HOLD = 200   # 끝에서 1초 정지

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
    # convexify=False: 신 에셋은 부품마다 `*_col`(원본 STL) 이 있어 링크 33개가
    # ABD 강체로 IPC 월드에 들어가는데, 기본 경로면 RightSlider1_1/Shaft_copy_1 의
    # 충돌 메시가 열린 채로 uipc 에 넘어가 빌드가 죽는다(AffineBodyConstitution ->
    # compute_mesh_volume: "Calculating volume of open trimesh is meaningless").
    # convexify=False 는 Genesis 의 watertighten(기본 5) 경로를 타서 33개 전부
    # 닫힌 비볼록 메시가 된다(실측: 열린 geom 0개).
    recovery = scene.add_entity(
        gs.morphs.MJCF(file=RECOVERY2_MJCF, pos=RECOVERY2_POS, decimate=False, convexify=False),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
    )

    cam = None
    if not use_viewer:
        cam = scene.add_camera(res=(1280, 960), pos=CAM_POS, lookat=CAM_LOOK, fov=45, GUI=False)

    print("\n[build] scene.build() 시작...")
    scene.build(n_envs=0)
    print(f"[build] 성공  fixture n_dofs={fixture.n_dofs}  recovery2 n_dofs={recovery.n_dofs}")

    jig_geoms = [g for g in fixture.geoms if g.link.name == "ServoShaft_1" and g.contype == 1]
    # Jig 포크가 실제로 밀어야 할 대상 = ShaftHandle_1. 구 에셋에서는 이 부품
    # 하나만 contype=1 이라 `g.contype == 1` 로 골라도 같은 결과였지만, 신 에셋은
    # 33개 부품이 전부 충돌 geom 을 가지므로 반드시 링크 이름으로 골라야 한다.
    # whole-body 시각 AABB는 참고용으로만 같이 본다(M_Bottom_1/Shaft_copy_1과
    # 겹치는 건 의도된 근접이라 더 이상 "버그"가 아님).
    sh_geoms = [g for g in recovery.geoms if g.link.name == "ShaftHandle_1"]
    rec_aabb = recovery.get_vAABB()
    rec_aabb = rec_aabb.cpu().numpy() if hasattr(rec_aabb, "cpu") else np.asarray(rec_aabb)

    def _npy(x):
        return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)

    def _aabb_overlap(a, b):
        return all(a[0][i] <= b[1][i] and b[0][i] <= a[1][i] for i in range(3))

    def check_overlap():
        n_body = 0
        n_handle = 0
        for g in jig_geoms:
            bb = _npy(g.get_AABB())
            if _aabb_overlap(bb, rec_aabb):
                n_body += 1
            for sh in sh_geoms:
                shbb = _npy(sh.get_AABB())
                if _aabb_overlap(bb, shbb):
                    n_handle += 1
        return n_body, n_handle

    if cam is not None:
        # Genesis 1.3.x: 파일명/fps 가 start_recording 으로 옮겨졌고 stop_recording()
        # 은 인자를 안 받는다(full_workflow.py 와 동일하게 이전).
        cam.start_recording(save_to_filename=MP4, fps=30)

    print(f"\n[spin] Servo3_ServoShaft 0 -> 2pi ({N_SPIN}스텝) — Jig 포크와 ShaftHandle 접촉 확인")
    for k in range(N_SPIN):
        q = 2 * np.pi * (k + 1) / N_SPIN
        fixture.set_dofs_position(np.array([q]))
        scene.step()          # 녹화 프레임은 이 안에서 잡힌다(별도 render 불필요)
        if k % 200 == 0:
            n_body, n_handle = check_overlap()
            print(f"    k={k:4d} q={q:.3f}rad  body겹침(참고용)={n_body}/{len(jig_geoms)}  "
                  f"ShaftHandle접촉={n_handle}/{len(jig_geoms)}"
                  f"{' (요 회전 미조정 - 아직 안 닿음, 정상)' if n_handle == 0 else ''}")

    print(f"[hold] 정지 유지 ({N_HOLD}스텝)")
    for k in range(N_HOLD):
        scene.step()

    if cam is not None:
        cam.stop_recording()
        print(f"\n[saved] {MP4}")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

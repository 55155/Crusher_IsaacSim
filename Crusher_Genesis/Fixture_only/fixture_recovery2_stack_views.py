"""
fixture_recovery2_stack_views.py — 고정장치 위에 회수장치2를 정렬 배치한
조립 상태를, 로봇/Crusher 없이 여러 각도의 정지 스냅샷으로만 확인.

배치(2026-07-27, 사용자 지시): 두 장치 각각의 원점(0,0,0) 기준 상대좌표로 준
정렬점을 world 좌표에서 일치시킨다:
  고정장치 쪽: (-138.00, 72.50, 151.30)mm = ServoShaft 중심
  회수장치2 쪽: (-74.015, 59.22, 1.776)mm
기구적으로 두 점이 정확히 한 점에서 만날 순 없어(사용자 지시) Y,Z만 맞으면
충분하지만, X도 함께 맞춰서 3축 다 일치시켰다(full_workflow.py의
RECOVERY2_POS 계산과 동일 로직, 여기서는 FIXTURE_POS=(0,0,0.12) 기준으로
재계산).

**에셋 교체(2026-08-19)**: 회수장치2_description/회수장치2.xml → fusion2xml
(MJCF 직행) 빌드인 recovery2_mjcf/recovery2.xml (대조표는 그 폴더 README.md).
RECOVERY2_POS 는 그대로 둔다 — 정렬점 (-74.015, 59.22, 1.776)mm 는 Fusion 조인트
앵커 값인데 구 빌드는 자기 샤프트 힌지축이 그 앵커에서 (+30.92, -6.62, 0)mm
어긋나 있었고 신 빌드는 0.01mm 안에서 일치한다. 즉 **같은 POS 로 회수장치2
형상이 고정장치 기준 (+30.92, -6.62, 0)mm 옮겨 앉으며, 그래야 원래 의도한
"Jig 포크가 ShaftHandle 을 돌린다"는 정렬이 실제로 성립한다.** 즉 예전 스냅샷과 비교하면 회수장치2가 그만큼 옮겨 보인다.

출력: RESULT/stack_view_<name>.png (corner/front/top45/topdown 4장)
"""
import os, sys
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

FIXTURE_MJCF = os.path.join(paths.ROBOTS_DIR, "고정장치_description", "고정장치.xml")
FIXTURE_POS = (0.0, 0.0, 0.12)

RECOVERY2_MJCF = os.path.join(paths.ROBOTS_DIR, "recovery2_mjcf", "recovery2.xml")
# world_target = FIXTURE_POS + (-0.138, 0.0725, 0.1513) = (-0.138, 0.0725, 0.2713)
# RECOVERY2_POS = world_target - (-0.074015, 0.05922, 0.001776)
# 2026-07-29: +25mm 인위적 간격 제거 - Jig는 담아 고정하는 컵이 아니라 기둥
# 2개로 ShaftHandle_1을 돌리는 포크 구조임을 확인(full_workflow.py 주석 참고),
# ShaftHandle 자체는 Jig와 안 겹쳐서 원래 정렬점으로 복귀.
RECOVERY2_POS = (-0.063985, 0.01328, 0.269524)

DT = 5e-3
IPC_D_HAT = 1.0e-4

# 조립체(고정장치+회수장치2) 전체 bbox 중심 근사 - 고정장치 단독 LOOKAT(z=0.1406,
# fixture_only_views.py 참고)보다 회수장치2가 얹혀 더 높이 올라간 만큼 z를 올림.
LOOKAT = (-0.09, 0.03, 0.2)
R = 0.55
FOV = 45

_DIRS = {
    "corner":  (0.5, 0.5, 0.70710678),          # 앙각45 + 대각(45도 코너)
    "front":   (0.0, -1.0, 0.3),                 # 정면(-y) 약간 위에서
    "top45":   (0.0, -0.70710678, 0.70710678),   # 정면-윗면 중간 45도
    "topdown": (0.05, 0.05, 1.0),                 # 거의 수직 위에서
}
VIEWS = {name: tuple(np.array(d) * R) for name, d in _DIRS.items()}


def main():
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
        show_viewer=False,
    )

    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(coup_type="ipc_only"))
    scene.add_entity(
        gs.morphs.MJCF(file=FIXTURE_MJCF, pos=FIXTURE_POS, decimate=False),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
    )
    # convexify=False — 기본 경로면 RightSlider1_1/Shaft_copy_1 의 충돌 메시가
    # 열린 채 uipc 로 넘어가 빌드가 죽는다(compute_mesh_volume 어서션).
    # convexify=False 는 watertighten(기본 5) 을 태워 33개 전부 닫히게 한다.
    scene.add_entity(
        gs.morphs.MJCF(file=RECOVERY2_MJCF, pos=RECOVERY2_POS, decimate=False, convexify=False),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
    )

    cam = scene.add_camera(res=(1280, 960), pos=(0.5, -0.5, 0.5), lookat=LOOKAT, fov=FOV, GUI=False)

    print("[build] scene.build() 시작...")
    scene.build(n_envs=0)
    print("[build] 성공")

    scene.step()

    import imageio.v3 as iio
    for name, offset in VIEWS.items():
        cam_pos = tuple(np.array(LOOKAT) + np.array(offset))
        cam.set_pose(pos=cam_pos, lookat=LOOKAT, up=(0, 0, 1))
        r = cam.render()
        img = r[0] if isinstance(r, (tuple, list)) else r
        img = img.cpu().numpy() if hasattr(img, "cpu") else np.asarray(img)
        out_path = os.path.join(OUT_DIR, f"stack_view_{name}.png")
        iio.imwrite(out_path, img)
        print(f"[saved] {name}: cam_pos={np.round(cam_pos,3)} -> {out_path}")

    print("완료.")


if __name__ == "__main__":
    main()

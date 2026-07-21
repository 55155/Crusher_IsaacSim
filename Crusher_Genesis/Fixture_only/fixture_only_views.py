"""
fixture_only_views.py — 고정장치 단독, 확대된 3-앵글 스냅샷.

fixture_only.py와 같은 씬(바닥+고정장치 단독)을 만들고, 카메라를 조립체
바운딩박스(약 0.35×0.235×0.275m) 기준으로 충분히 당겨서(거리 0.5m) 3장을
찍는다:
  1. corner  : 꼭지점 뷰 — 정면(-y)+측면(+x) 대각선, 앙각 45도
  2. front   : 정면 뷰 — -y축에서 수평(앙각 0도)으로 정면
  3. top45   : 정면-윗면 중간 45도 — 정면(-y) 방향 유지한 채 앙각만 45도로 든 뷰

Jig 파트가 보이도록 Servo3_ServoShaft를 살짝 돌려놓은 자세(q=0.6rad)에서 촬영.

출력: RESULT/fixture_view_corner.png, _front.png, _top45.png
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
Q_POSE = 0.6  # Jig가 살짝 돌아간 자세로 촬영(0rad이면 Servo3와 겹쳐 안 보임)

DT = 5e-3
IPC_D_HAT = 1.0e-4

# 조립체 월드 bbox 중심(고정장치_description 메시로 계산, FIXTURE_POS 반영)
LOOKAT = (-0.045, 0.0275, 0.1406)
R = 0.7   # 카메라~lookat 거리(충분히 가까운 확대 샷)
FOV = 40

# 각 방향 = 단위벡터 * R (R을 곱해서 오프셋으로 사용)
_DIRS = {
    "corner": (0.5, 0.5, 0.70710678),      # 앙각45 + 정면·측면 대각(45도 코너)
    "front":  (0.0, -1.0, 0.0),             # -y축 정면, 앙각 0
    "top45":  (0.0, -0.70710678, 0.70710678),  # 정면 방향 유지, 앙각 45(정면-윗면 중간)
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
    fixture = scene.add_entity(
        gs.morphs.MJCF(file=FIXTURE_MJCF, pos=FIXTURE_POS, decimate=False),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
    )

    cam = scene.add_camera(res=(1280, 960), pos=(0.5, -0.5, 0.5), lookat=LOOKAT, fov=FOV, GUI=False)

    print("[build] scene.build() 시작...")
    scene.build(n_envs=0)
    print("[build] 성공")

    # 정지 자세로 살짝 회전(Jig가 눈에 띄게)
    for k in range(80):
        s = (k + 1) / 80
        fixture.set_dofs_position(np.array([Q_POSE * s]))
        scene.step()

    import imageio.v3 as iio
    for name, offset in VIEWS.items():
        cam_pos = tuple(np.array(LOOKAT) + np.array(offset))
        # up을 매번 명시해야 함 — 생략하면 set_pose()가 직전 호출에서 틀어진
        # up 벡터를 그대로 이어받아 화면이 기울어진다(Genesis camera.py:660).
        cam.set_pose(pos=cam_pos, lookat=LOOKAT, up=(0, 0, 1))
        r = cam.render()
        img = r[0] if isinstance(r, (tuple, list)) else r
        img = img.cpu().numpy() if hasattr(img, "cpu") else np.asarray(img)
        out_path = os.path.join(OUT_DIR, f"fixture_view_{name}.png")
        iio.imwrite(out_path, img)
        print(f"[saved] {name}: cam_pos={np.round(cam_pos,3)} -> {out_path}")

    print("완료.")


if __name__ == "__main__":
    main()

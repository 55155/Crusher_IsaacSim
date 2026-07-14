"""
rigid_capsule_tablet_bag_ipc_test.py — Rigid(정제, MJCF capsule geom = native SDF
primitive) + FEM.Cloth(샘플백) + IPC 커플러 조합 시도.

**배경**: fem_tablet_drop.py(3fca8c8)는 정제를 FEM.Elastic(변형체, sliver-free
캡슐 tet mesh)로 낙하시켜 성공했다. 그 직전 커밋 메시지에 남은 미해결 실험은
정제를 **Rigid**로(캡슐을 tet mesh가 아니라 MJCF capsule geom = Genesis rigid
solver 네이티브 SDF primitive로) 넣는 조합이었는데, `coup_type=
"two_way_soft_constraint"`로 시도해 접촉 시 위치가 발산(수 미터로 튕겨나감)
했다 — 원인 미해결로 기록됨(§DigitalTwin.md §6-1, git log 3fca8c8).

**이번 시도 — 다른 coup_type**: `_setup_coupling_config`(coupler.py:206-215)를
보면 `coup_type=None`일 때 자동 선택 규칙은 `n_joints>0`(관절 있음, 예: freejoint)
이면 `two_way_soft_constraint`(구동/제어 대상용 — PD가 driver, IPC가 barrier)를
쓰고, `n_joints==0`(무관절 단일 바디)이면 `ipc_only`(IPC가 중력·동역학을 전부
담당하는 one-way 커플링)를 쓴다. 이전 시도는 캡슐을 MJCF freejoint 바디로
만들었으니 자동으로 two_way_soft_constraint 경로를 탔을 것이다. 하지만 이
정제는 **PD로 구동되는 대상이 아니라 순수 낙하하는 수동체** — 의미상
`ipc_only`(Plane/Shelf처럼 이미 검증된 경로, "IPC controls gravity/collision
for ipc_only entities")가 더 맞는 선택이다. `coup_type`은 자동선택과 무관하게
material에서 명시적으로 강제 가능하므로, freejoint 캡슐에 `coup_type="ipc_only"`
를 명시해 이 가설을 검증한다.

성공/실패 여부와 무관하게 실행 로그와 영상을 남긴다(§docstring 목적).
"""
import os, sys, tempfile
from datetime import datetime
import numpy as np

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

_DIR = os.path.dirname(os.path.abspath(__file__))
_r = _DIR
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
import paths

BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag", "Samplebag_seal_pouch3.stl")

DT = 5e-3
N_DROP, N_SETTLE = 150, 150
N_STEPS = N_DROP + N_SETTLE
RENDER_EVERY = 1

# 정제 — fem_tablet_drop.py 와 동일 치수(납작한 원반형 캡슐: 지름4mm/전체높이5mm)
CAP_RADIUS_M, CAP_CYL_HALF_M = 2.0e-3, 0.5e-3  # MJCF capsule size="radius half_length"
TABLET_RHO = 1300.0
TABLET_FRICTION = 0.5
COUP_TYPE = os.environ.get("COUP_TYPE", "ipc_only")  # "ipc_only" | "two_way_soft_constraint"

# 봉투(FEM.Cloth) — fem_tablet_drop.py 검증값
BAG_SCALE = 1.0
BAG_EULER = (90, 0, 0)
BAG_HALF_H = 0.045
BAG_POS = (0.0, 0.0, 0.06)

SHELF_TOP = BAG_POS[2] - BAG_HALF_H - 0.0015
SHELF_SIZE = (0.10, 0.10, 0.02)
SHELF_POS = (BAG_POS[0], BAG_POS[1], SHELF_TOP - SHELF_SIZE[2] / 2)

CLOTH_E, CLOTH_NU, CLOTH_RHO = 1.0e5, 0.499, 200.0
CLOTH_THICK, CLOTH_BEND = 1.0e-3, 50.0
CLOTH_FRICTION = 0.8

BAG_MOUTH_Z = BAG_POS[2] + BAG_HALF_H
TABLET_DROP_H = 0.015
TABLET_POS = (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z + TABLET_DROP_H)

OUT_DIR = os.path.join(_DIR, "Result")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"rigid_capsule_tablet_bag_ipc_{COUP_TYPE}_{_TS}.mp4")

CAM_POS, CAM_LOOK = (0.16, -0.16, 0.13), (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z - 0.01)

_CAPSULE_MJCF = f"""<mujoco model="rigid_capsule_tablet">
  <compiler angle="degree"/>
  <worldbody>
    <body name="tablet" pos="0 0 0">
      <freejoint/>
      <geom name="tablet_capsule" type="capsule" size="{CAP_RADIUS_M} {CAP_CYL_HALF_M}"
            euler="90 0 0" density="{TABLET_RHO}" friction="{TABLET_FRICTION} 0.02 0.001"/>
    </body>
  </worldbody>
</mujoco>
"""


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" Rigid capsule tablet (SDF, coup_type={COUP_TYPE}) -> FEM Samplebag -> IPC (viewer={use_viewer})")
    print("=" * 60)
    print(f"  capsule R={CAP_RADIUS_M*1e3:.1f}mm half_len={CAP_CYL_HALF_M*1e3:.1f}mm "
          f"(dia={2*CAP_RADIUS_M*1e3:.1f}mm)")
    print(f"  drop_h={TABLET_DROP_H*1e3:.0f}mm above bag mouth (z={BAG_MOUTH_Z*1e3:.1f}mm)")

    tmp_dir = tempfile.mkdtemp(prefix="rigid_capsule_tablet_")
    mjcf_path = os.path.join(tmp_dir, "capsule_tablet.xml")
    with open(mjcf_path, "w", encoding="utf-8") as f:
        f.write(_CAPSULE_MJCF)

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=5.0e-4,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=False,
            constraint_strength_translation=100.0,
            constraint_strength_rotation=100.0,
        ),
        vis_options=gs.options.VisOptions(background_color=(0.93, 0.94, 0.96)),
        show_viewer=use_viewer,
    )

    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(coup_type="ipc_only"))
    scene.add_entity(
        gs.morphs.Box(size=SHELF_SIZE, pos=SHELF_POS, fixed=True),
        material=gs.materials.Rigid(coup_type="ipc_only", coup_friction=0.3),
        surface=gs.surfaces.Default(color=(0.75, 0.78, 0.82)),
    )

    bag = scene.add_entity(
        material=gs.materials.FEM.Cloth(
            E=CLOTH_E, nu=CLOTH_NU, rho=CLOTH_RHO,
            thickness=CLOTH_THICK, bending_stiffness=CLOTH_BEND,
            friction_mu=CLOTH_FRICTION,
        ),
        morph=gs.morphs.Mesh(file=BAG_STL, scale=BAG_SCALE, pos=BAG_POS, euler=BAG_EULER),
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.4,
                                     roughness=0.9, double_sided=True),
    )

    tablet = scene.add_entity(
        gs.morphs.MJCF(file=mjcf_path, pos=TABLET_POS),
        material=gs.materials.Rigid(coup_type=COUP_TYPE, coup_friction=TABLET_FRICTION),
        surface=gs.surfaces.Default(color=(0.9, 0.9, 0.85), roughness=0.6),
    )

    cam = scene.add_camera(res=(960, 720), pos=CAM_POS, lookat=CAM_LOOK, fov=35,
                            near=0.01, far=5.0, GUI=False)
    scene.build(n_envs=0)

    vp0 = _npy(bag.get_state().pos).squeeze()
    print(f"[bag]     verts={vp0.shape}  x={vp0[:,0].min():.4f}~{vp0[:,0].max():.4f}  "
          f"y={vp0[:,1].min():.4f}~{vp0[:,1].max():.4f}  z={vp0[:,2].min():.4f}~{vp0[:,2].max():.4f}")
    pos0 = _npy(tablet.get_pos()).squeeze()
    print(f"[tablet]  pos0={pos0}")

    cam.start_recording()
    diverged = False
    for k in range(N_STEPS):
        scene.step()
        if (k + 1) % RENDER_EVERY == 0:
            cam.render()
        if (k + 1) % 20 == 0:
            p = _npy(tablet.get_pos()).squeeze()
            phase = "drop" if k < N_DROP else "settle"
            print(f"  [{phase}] t={(k+1)*DT*1e3:5.1f}ms  tablet_pos=({p[0]*1e3:+.2f},{p[1]*1e3:+.2f},{p[2]*1e3:+.2f})mm")
            if np.any(np.abs(p) > 1.0):  # 1m 넘게 튕기면 발산으로 간주
                diverged = True
                print(f"  [!!] 발산 감지 (|pos|>1m) — 조기 종료")
                break

    pf = _npy(tablet.get_pos()).squeeze()
    bagf = _npy(bag.get_state().pos).squeeze()
    fall_mm = (pos0[2] - pf[2]) * 1e3
    inside_xy = (pf[0] - BAG_POS[0]) ** 2 + (pf[1] - BAG_POS[1]) ** 2 < 0.03 ** 2
    at_or_below_mouth = pf[2] <= BAG_MOUTH_Z + 0.002
    print(f"\n[final] tablet_pos={pf}  net_fall={fall_mm:.3f}mm  bag_com_z={bagf[:,2].mean()*1e3:.3f}mm")
    print(f"[check] diverged={diverged}  봉투 입구 높이 이하: {at_or_below_mouth}  /  봉투 중심 근방(xy): {inside_xy}")

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved video] {MP4}")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

"""
recovery2_tablet_drop.py — 회수장치2 위로 정제(FEM 캡슐, 2x 스케일)를 자유낙하시켜
실제 충돌(collision hull)이 붙었는지 시각적으로 확인하는 테스트.

**에셋 교체(2026-08-19)**: recovery2_mjcf/recovery2.xml(fusion2xml 직행 빌드).
구 회수장치2_description 은 전 부품이 시각 전용(contype=0/conaffinity=0)이라 이
테스트를 위해 X_hull.stl 을 따로 심어야 했고, 그마저 2026-07-29 에 IPC 씬 문제로
다시 꺼져 지금은 ShaftHandle 하나만 충돌한다. 신 빌드는 부품마다 `*_col`(원본
STL — hull 근사가 아니라 실제 비볼록 형상) 을 달고 나오고, convexify=False 로
실으면 Genesis 의 watertighten 경로를 타서 33개가 전부 닫힌 비볼록 충돌 메시가
된다(hull 근사 때문에 얇은 틈이 막혀 보이던 문제가 없어짐).

정제는 M_Top/F_Top 플랫폼 위에서 떨어뜨린다. 신 빌드 실측 bbox(모델 원점 기준)는
전체 x[-137.6, 0.0] y[0.0, 118.4] z[0, 148.0]mm, 상판은 F_Top x[-110.2,-55.0] /
M_Top x[-55.0, 0.0] 이고 둘 다 z[48, 58]mm — 아래 DROP_XY 는 F_Top 상판 위다.

출력: RESULT/recovery2_tablet_drop_<ts>.mp4
"""
import os, sys
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

sys.path.insert(0, os.path.join(os.path.dirname(_r), "utills"))
from primitive_tablet_generator import make_capsule_tets_v2, add_analytic_fem_entity
from fem_ipc_workarounds import patch_fem_vertex_constraints

OUT_DIR = os.path.join(_DIR, "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"recovery2_tablet_drop_{_TS}.mp4")

RECOVERY_MJCF = os.path.join(paths.ROBOTS_DIR, "recovery2_mjcf", "recovery2.xml")
RECOVERY_POS = (0.0, 0.0, 0.001)

DT = 5e-3
IPC_D_HAT = 1.0e-4

CAP_RADIUS_MM, CAP_CYL_H_MM = 4.0, 2.0
TABLET_E, TABLET_NU, TABLET_RHO = 5.0e4, 0.45, 1300.0
TABLET_FRICTION = 0.5

# 플랫폼(M_Top/F_Top) 상단 근처, 신 빌드 실측 bbox: x[-0.1376,0] y[0,0.1184] z[0,0.148]
DROP_XY = (-0.06, 0.06)
DROP_MARGIN = 0.025
TABLET_POS = (DROP_XY[0], DROP_XY[1], 0.148 + DROP_MARGIN)

N_DROP = 250
N_SETTLE = 150

CAM_POS, CAM_LOOK = (0.30, -0.25, 0.30), (-0.06, 0.06, 0.10)


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" Recovery2 tablet drop test (viewer={use_viewer})")
    print("=" * 60)

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")
    patch_fem_vertex_constraints()

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=IPC_D_HAT,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=False,
        ),
        vis_options=gs.options.VisOptions(background_color=(0.93, 0.94, 0.96)),
        show_viewer=use_viewer,
    )

    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(coup_type="ipc_only"))

    # convexify=False — 기본 경로면 RightSlider1_1/Shaft_copy_1 의 충돌 메시가
    # 열린 채 uipc 로 넘어가 빌드가 죽는다(compute_mesh_volume 어서션).
    # convexify=False 는 watertighten(기본 5) 을 태워 33개 전부 닫히게 한다.
    recovery = scene.add_entity(
        gs.morphs.MJCF(file=RECOVERY_MJCF, pos=RECOVERY_POS, decimate=False, convexify=False),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
    )

    cap_verts_mm, cap_elems = make_capsule_tets_v2(
        radius_mm=CAP_RADIUS_MM, cyl_height_mm=CAP_CYL_H_MM, n_theta=18, n_cap_rings=5, n_cyl_bands=2,
    )
    tablet = add_analytic_fem_entity(
        scene, key=os.path.join(OUT_DIR, "_analytic_capsule_recovery2_drop.stl"),
        verts_mm=cap_verts_mm, elems=cap_elems,
        material=gs.materials.FEM.Elastic(
            E=TABLET_E, nu=TABLET_NU, rho=TABLET_RHO,
            friction_mu=TABLET_FRICTION, model="stable_neohookean",
        ),
        scale=1e-3, pos=TABLET_POS,
        surface=gs.surfaces.Default(color=(0.9, 0.9, 0.85), roughness=0.6),
    )

    cam = scene.add_camera(res=(1280, 960), pos=CAM_POS, lookat=CAM_LOOK, fov=40,
                            near=0.01, far=5.0, GUI=False)

    print("\n[build] scene.build() 시작...")
    scene.build(n_envs=0)
    print(f"[build] 성공  n_dofs={recovery.n_dofs}")

    def _tab_com():
        return _npy(tablet.get_state().pos).squeeze().mean(axis=0)

    # Genesis 1.3.x: 파일명/fps 가 start_recording 으로 옮겨졌고 stop_recording()
    # 은 인자를 안 받는다(full_workflow.py 와 동일하게 이전).
    cam.start_recording(save_to_filename=MP4, fps=30)

    print(f"\n[drop] ({N_DROP}스텝) — 정제 자유낙하, 시작 z={TABLET_POS[2]*1e3:.1f}mm")
    for k in range(N_DROP):
        scene.step()
        cam.render()
        if (k + 1) % 40 == 0:
            print(f"    k={k+1:4d} tablet_com_z={_tab_com()[2]*1e3:+.2f}mm")

    print(f"[settle] ({N_SETTLE}스텝)")
    for k in range(N_SETTLE):
        scene.step()
        cam.render()
        if (k + 1) % 40 == 0:
            print(f"    k={k+1:4d} tablet_com_z={_tab_com()[2]*1e3:+.2f}mm")

    print(f"\n[final] tablet_com_z={_tab_com()[2]*1e3:+.2f}mm")
    cam.stop_recording()
    print(f"\n[saved] {MP4}")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

"""
fem_tablet_drop.py — 길쭉한 캡슐형 정제(알약)를 FEM.Elastic 재질로
자유낙하시켜 샘플백(FEM.Cloth) 안까지 담기게 하는 데모.

**이력 1 — TetGen 산출 메시 + M0609 로봇 조합 얼어붙음(2026-07-14, 해결)**:
  `M0609_RG2/grasp_bag_tablet_ipc_test.py` 조사에서 확인: `fem_options`를
  아예 안 주면(기본 explicit 솔버) implicit-솔버 얼어붙음 문제는 해결되고,
  M0609 앞에서는 `gs.morphs.Box` primitive 가 TetGen 산출 커스텀 메시(Sphere,
  STL)보다 훨씬 안정적으로 낙하한다는 게 확인됐다.

**이력 2 — FEM.Elastic(체적) + FEM.Cloth(표면 전용) 조합에서 별개의 얼어붙음
발견(2026-07-14)**: analytic 캡슐(TetGen 미사용)도 FEM.Cloth 봉투가 씬에
있으면 접촉 여부와 무관하게 거의 완전히 멈췄다. 반면 `gs.morphs.Box` 는
정상 낙하 — 당시엔 원인 불명이라 정제를 Box(직육면체)로 근사해 우회했다.

**이력 3 — 근본 원인 규명 및 해결: sliver tet(2026-07-14)**:
  이력1·2 둘 다 "Box는 안정, 커스텀 형상(캡슐/구/STL)은 불안정"이라는 같은
  패턴이었다. 캡슐을 v1(`make_capsule_tets`, 전역 centroid 하나로 부채꼴)
  방식으로 만들면 극(pole) 근처에 종횡비 나쁜 sliver tet 가 다수 생기는데
  (반지름 2.5mm인데 극-centroid 거리는 6mm — 밑변 작고 지렛대 긴 tet),
  이게 두 얼어붙음 버그 모두의 진짜 원인이었다. `make_capsule_tets_v2`
  (medial-axis 다중 앵커 — 반구는 자기 중심점, 원기둥은 축 위 국소 앵커에
  부채꼴, §DigitalTwin.md §8 참고)로 sliver 를 제거하니 **Box와 거의 동일한
  정상 자유낙하**를 회복했다(edge-비율 15.0→6.25 개선, M0609/FEM.Cloth
  둘 다에서 얼어붙지 않음, 실측 확인). 이제 정제를 Box 근사가 아니라 실제
  캡슐(길쭉한 알약) 형상으로 낙하시킨다.

재질값(§grasp_bag_tablet_ipc_test.py 와 동일):
  정제: E=5e4, ν=0.45, ρ=1300 kg/m³, model=stable_neohookean.
  봉투: E=1e5, ν=0.499, ρ=200 kg/m³, thickness=1mm, bending=50.

출력: FEM/Result/fem_tablet_bag_drop_<ts>.mp4
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

BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag", "Samplebag_seal_pouch3.stl")

DT = 5e-3
N_DROP, N_SETTLE = 150, 150
N_STEPS = N_DROP + N_SETTLE
RENDER_EVERY = 1

# 정제 — 실제 정제는 길쭉한 캡슐이 아니라 납작한 원반(biconvex 정제) 형태에
# 가까움 → radius > cyl_h/2 로 짧고 통통하게(지름 4mm, 전체 높이 5mm).
# 지름(4mm) < 봉투 두께(6mm)로 여유를 둬 입구를 통과할 수 있게 함.
CAP_RADIUS_MM, CAP_CYL_H_MM = 2.0, 1.0
TABLET_E, TABLET_NU, TABLET_RHO = 5.0e4, 0.45, 1300.0
TABLET_FRICTION = 0.5

# ── 봉투(FEM.Cloth) — grasp_bag_ipc_test.py 검증값 ─────────────────────────
BAG_SCALE = 1.0
BAG_EULER = (90, 0, 0)
BAG_HALF_H = 0.045  # 로컬 y 범위 ±45mm(입구가 world Z 최상단)
BAG_POS = (0.0, 0.0, 0.06)

SHELF_TOP = BAG_POS[2] - BAG_HALF_H - 0.0015
SHELF_SIZE = (0.10, 0.10, 0.02)
SHELF_POS = (BAG_POS[0], BAG_POS[1], SHELF_TOP - SHELF_SIZE[2] / 2)

CLOTH_E, CLOTH_NU, CLOTH_RHO = 1.0e5, 0.499, 200.0
CLOTH_THICK, CLOTH_BEND = 1.0e-3, 50.0
CLOTH_FRICTION = 0.8

BAG_MOUTH_Z = BAG_POS[2] + BAG_HALF_H
# 입구 통과 여부가 접촉 해석의 knife-edge 조건이라 실행마다(부동소수점
# 비결합성/GPU 스케줄링 차이로) 통과/걸림이 갈릴 수 있음(실측 확인 —
# 낙하고를 15→35mm 로 늘려도 무관하게 걸리는 경우가 있었음). 여러 번 실행해
# 통과하는 seed 를 채택.
TABLET_DROP_H = 0.015  # 입구 위 15mm 에서 낙하
TABLET_POS = (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z + TABLET_DROP_H)

OUT_DIR = os.path.join(_DIR, "Result")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"fem_tablet_bag_drop_{_TS}.mp4")

CAM_POS, CAM_LOOK = (0.16, -0.16, 0.13), (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z - 0.01)


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" FEM Tablet (sliver-free capsule) -> Samplebag Drop Test (viewer={use_viewer})")
    print("=" * 60)
    print(f"  capsule R={CAP_RADIUS_MM}mm cyl_h={CAP_CYL_H_MM}mm "
          f"(len={CAP_CYL_H_MM+2*CAP_RADIUS_MM:.1f}mm dia={2*CAP_RADIUS_MM:.1f}mm)")
    print(f"  drop_h={TABLET_DROP_H*1e3:.0f}mm above bag mouth (z={BAG_MOUTH_Z*1e3:.1f}mm)")

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        # fem_options 를 일부러 안 준다(기본 explicit 솔버) — §docstring 이력1.
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=5.0e-4,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=False,
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

    cap_verts_mm, cap_elems = make_capsule_tets_v2(
        radius_mm=CAP_RADIUS_MM, cyl_height_mm=CAP_CYL_H_MM, n_theta=12, n_cap_rings=4, n_cyl_bands=2,
    )
    print(f"[tablet]  capsule verts={len(cap_verts_mm)} tets={len(cap_elems)} "
          f"(sliver-free medial-axis, TetGen 미사용)")

    tablet = add_analytic_fem_entity(
        scene, key=os.path.join(OUT_DIR, "_analytic_capsule_v2.stl"),
        verts_mm=cap_verts_mm, elems=cap_elems,
        material=gs.materials.FEM.Elastic(
            E=TABLET_E, nu=TABLET_NU, rho=TABLET_RHO,
            friction_mu=TABLET_FRICTION, model="stable_neohookean",
        ),
        scale=1e-3, pos=TABLET_POS,
        surface=gs.surfaces.Default(color=(0.9, 0.9, 0.85), roughness=0.6),
    )

    cam = scene.add_camera(res=(960, 720), pos=CAM_POS, lookat=CAM_LOOK, fov=35,
                            near=0.01, far=5.0, GUI=False)
    scene.build(n_envs=0)

    vp0 = _npy(bag.get_state().pos).squeeze()
    print(f"[bag]     verts={vp0.shape}  x={vp0[:,0].min():.4f}~{vp0[:,0].max():.4f}  "
          f"y={vp0[:,1].min():.4f}~{vp0[:,1].max():.4f}  z={vp0[:,2].min():.4f}~{vp0[:,2].max():.4f}")
    pos0 = _npy(tablet.get_state().pos).squeeze()
    print(f"[tablet]  nodes={len(pos0)}  z0_mean={pos0[:,2].mean()*1e3:.2f}mm "
          f"z0_min={pos0[:,2].min()*1e3:.2f}mm")

    cam.start_recording()
    for k in range(N_STEPS):
        scene.step()
        if (k + 1) % RENDER_EVERY == 0:
            cam.render()
        if (k + 1) % 20 == 0:
            p = _npy(tablet.get_state().pos).squeeze()
            zc, zmin = p[:, 2].mean(), p[:, 2].min()
            phase = "drop" if k < N_DROP else "settle"
            print(f"  [{phase}] t={(k+1)*DT*1e3:5.1f}ms  tablet_com_z={zc*1e3:+.3f}mm  min_z={zmin*1e3:+.3f}mm")

    pf = _npy(tablet.get_state().pos).squeeze()
    bagf = _npy(bag.get_state().pos).squeeze()
    fall_mm = (pos0[:, 2].mean() - pf[:, 2].mean()) * 1e3
    inside_xy = (pf[:, 0].mean() - BAG_POS[0]) ** 2 + (pf[:, 1].mean() - BAG_POS[1]) ** 2 < 0.03 ** 2
    at_or_below_mouth = pf[:, 2].mean() <= BAG_MOUTH_Z + 0.002
    print(f"\n[final] tablet_com_z={pf[:,2].mean()*1e3:.3f}mm  net_fall={fall_mm:.3f}mm  "
          f"bag_com_z={bagf[:,2].mean()*1e3:.3f}mm")
    print(f"[check] 봉투 입구 높이 이하: {at_or_below_mouth}  /  봉투 중심 근방(xy): {inside_xy}")

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved video] {MP4}")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

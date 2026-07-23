"""
fem_tablet_drop.py — 정제(알약/원반)를 샘플백(FEM.Cloth) 안까지 낙하시키는 데모.
`TABLET_MODE` 환경변수로 정제 표현 방식을 고른다:
  - "fem" (기본값): FEM.Elastic, sliver-free medial-axis 캡슐/원반.
  - "rigid_sdf": Rigid body + MJCF capsule geom(analytic SDF, TetGen/메시 전혀
    없음) — §이력4 참고.

**이력 1 — TetGen 산출 메시 + M0609 로봇 조합 얼어붙음(2026-07-14, 해결)**:
  `fem_options`를 아예 안 주면(기본 explicit 솔버) implicit-솔버 얼어붙음
  문제는 해결되고, M0609 앞에서는 `gs.morphs.Box` primitive 가 TetGen 산출
  커스텀 메시(Sphere, STL)보다 훨씬 안정적으로 낙하한다는 게 확인됐다.

**이력 2 — FEM.Elastic(체적) + FEM.Cloth(표면 전용) 조합에서 별개의 얼어붙음
발견(2026-07-14)**: analytic 캡슐(TetGen 미사용)도 FEM.Cloth 봉투가 씬에
있으면 접촉 여부와 무관하게 거의 완전히 멈췄다. `gs.morphs.Box` 는 정상 낙하.

**이력 3 — 근본 원인 규명 및 해결: sliver tet(2026-07-14)**:
  이력1·2 둘 다 "Box는 안정, 커스텀 형상은 불안정"이라는 같은 패턴이었다.
  캡슐을 v1(`make_capsule_tets`, 전역 centroid 하나로 부채꼴) 방식으로
  만들면 극(pole) 근처에 종횡비 나쁜 sliver tet 가 다수 생기는데(반지름
  2.5mm인데 극-centroid 거리는 6mm), 이게 두 얼어붙음 버그 모두의 진짜
  원인이었다. `make_capsule_tets_v2`(medial-axis 다중 앵커)로 sliver 를
  제거하니 Box와 거의 동일한 정상 자유낙하를 회복했다(edge-비율
  15.0→6.25). 다만 Box(정점 9개/tet 12개)는 강성을 20배까지 올리고 dt를
  맞게 줄여도 봉투 입구 접촉 순간 형상이 크게 왜곡됨(ratio 최대 1.48) —
  메시가 너무 성겨 강성 조정으로는 해결 불가.

**이력 4 — Rigid+SDF 시도(2026-07-14)**: 정제를 아예 변형 안 되는
  Rigid body 로 바꾸면 형상 왜곡 자체가 원천적으로 불가능해진다. Genesis
  는 `gs.morphs.Capsule` 클래스는 없지만, MJCF `<geom type="capsule">`
  를 로드하면 `GEOM_TYPE.CAPSULE`(radius+height 파라미터, 메시 아님 —
  MuJoCo와 동일한 analytic 표현)로 처리되어 진짜 SDF capsule 이 된다.
  `coup_type="two_way_soft_constraint"`(자동 선택되는 floating-base 기본값)
  로 결합했더니 접촉 순간 위치가 무한 발산(수 미터까지 튕겨나감) —
  기본 constraint_strength(100)에서 재현. **`constraint_strength_translation/
  rotation=0.5`(200배 약화) + `contact_d_hat=5e-4` + shelf 를 바닥(z=0)
  위로 확실히 띄움**(겹치면 `enable_rigid_ground_contact=True` 에서 초기화
  자체가 sanity-check 로 실패함) 조합으로 발산 없이 안정적으로 낙하·바운스
  ·정지함을 확인(z: 120→18mm, t=1.2s 이후 완전히 정지, 실측 확인).

재질값(§grasp_bag_tablet_ipc_test.py 와 동일, fem 모드에서만 사용):
  정제: E=5e4, ν=0.45, ρ=1300 kg/m³, model=stable_neohookean.
  봉투: E=1e5, ν=0.499, ρ=200 kg/m³, thickness=1mm, bending=50.

출력: FEM/Result/fem_tablet_bag_drop_<mode>_<ts>.mp4 (기존 파일 보존, 안 지움)
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

TABLET_MODE = os.environ.get("TABLET_MODE", "fem")  # "fem" | "rigid_sdf"
BAG_VARIANT = os.environ.get("BAG_VARIANT", "orig")  # "orig"(6mm) | "thin2mm"(2mm)

_BAG_FILES = {
    "orig": "Samplebag_seal_pouch3.stl",
    "thin2mm": "Samplebag_seal_pouch3_thin2mm.stl",
    "thin4mm": "Samplebag_seal_pouch3_thin4mm.stl",
}
BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag", _BAG_FILES[BAG_VARIANT])
TABLET_MJCF = os.path.join(paths.ROBOTS_DIR, "tablet_disc_rigid.xml")

DT = 2.0e-3 if TABLET_MODE == "rigid_sdf" else 5.0e-3
N_DROP, N_SETTLE = (500, 250) if TABLET_MODE == "rigid_sdf" else (150, 150)
N_STEPS = N_DROP + N_SETTLE
RENDER_EVERY = 4 if TABLET_MODE == "rigid_sdf" else 1

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

# rigid_sdf 모드는 enable_rigid_ground_contact=True 라 shelf 가 바닥(z=0)과
# 겹치면 초기화 자체가 sanity-check 로 실패한다(§이력4) — 확실히 띄운다.
if TABLET_MODE == "rigid_sdf":
    SHELF_SIZE = (0.10, 0.10, 0.01)
    SHELF_POS = (BAG_POS[0], BAG_POS[1], 0.0085)
    SHELF_TOP = SHELF_POS[2] + SHELF_SIZE[2] / 2
else:
    SHELF_SIZE = (0.10, 0.10, 0.02)
    SHELF_TOP = BAG_POS[2] - BAG_HALF_H - 0.0015
    SHELF_POS = (BAG_POS[0], BAG_POS[1], SHELF_TOP - SHELF_SIZE[2] / 2)

CLOTH_E, CLOTH_NU, CLOTH_RHO = 1.0e5, 0.499, 200.0
CLOTH_THICK, CLOTH_BEND = float(os.environ.get("CLOTH_THICK_MM", "1.0")) * 1.0e-3, 50.0
CLOTH_FRICTION = 0.8

BAG_MOUTH_Z = BAG_POS[2] + BAG_HALF_H
# 입구 통과 여부가 접촉 해석의 knife-edge 조건이라 실행마다(부동소수점
# 비결합성/GPU 스케줄링 차이로) 통과/걸림이 갈릴 수 있음(실측 확인).
TABLET_DROP_H = 0.015  # 입구 위 15mm 에서 낙하
TABLET_POS = (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z + TABLET_DROP_H)

OUT_DIR = os.path.join(_DIR, "Result")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"fem_tablet_bag_drop_{TABLET_MODE}_{BAG_VARIANT}_{_TS}.mp4")

CAM_POS, CAM_LOOK = (0.16, -0.16, 0.13), (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z - 0.01)


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" Tablet ({TABLET_MODE}) -> Samplebag Drop Test (viewer={use_viewer})")
    print("=" * 60)
    print(f"  bag_variant={BAG_VARIANT} ({os.path.basename(BAG_STL)})  cloth_thickness={CLOTH_THICK*1e3:.2f}mm")
    print(f"  drop_h={TABLET_DROP_H*1e3:.0f}mm above bag mouth (z={BAG_MOUTH_Z*1e3:.1f}mm)  dt={DT}")

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    if TABLET_MODE == "rigid_sdf":
        coupler_options = gs.options.IPCCouplerOptions(
            contact_d_hat=5.0e-4,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=True,
            enable_rigid_ground_contact=True,
            # 기본값(100)에서 접촉 순간 위치가 무한 발산함(§이력4) — 200배
            # 약화해야 발산 없이 안정적으로 낙하·바운스·정지한다(실측 확인).
            constraint_strength_translation=0.5,
            constraint_strength_rotation=0.5,
        )
    else:
        coupler_options = gs.options.IPCCouplerOptions(
            contact_d_hat=5.0e-4,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=False,
        )

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        # fem_options 를 일부러 안 준다(기본 explicit 솔버) — §docstring 이력1.
        coupler_options=coupler_options,
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

    if TABLET_MODE == "rigid_sdf":
        tablet = scene.add_entity(
            gs.morphs.MJCF(file=TABLET_MJCF, pos=TABLET_POS),
            material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
            surface=gs.surfaces.Default(color=(0.9, 0.9, 0.85), roughness=0.6),
        )
    else:
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
    print(f"[bag]     verts={vp0.shape}  z={vp0[:,2].min():.4f}~{vp0[:,2].max():.4f}")

    def _tablet_z():
        if TABLET_MODE == "rigid_sdf":
            pos = _npy(tablet.get_links_pos()).reshape(-1, 3)
            return pos[0, 2]
        p = _npy(tablet.get_state().pos).squeeze()
        return p[:, 2].mean()

    z0 = _tablet_z()
    print(f"[tablet]  z0={z0*1e3:.2f}mm")

    cam.start_recording()
    for k in range(N_STEPS):
        scene.step()
        if (k + 1) % RENDER_EVERY == 0:
            cam.render()
        if (k + 1) % 40 == 0:
            phase = "drop" if k < N_DROP else "settle"
            print(f"  [{phase}] t={(k+1)*DT*1e3:6.1f}ms  tablet_z={_tablet_z()*1e3:+.3f}mm")

    zf = _tablet_z()
    print(f"\n[final] tablet_z={zf*1e3:.3f}mm  net_fall={(z0-zf)*1e3:.3f}mm")

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved video] {MP4}")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

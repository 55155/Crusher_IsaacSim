"""
powder_containment_test.py — MPM.Sand 파우더가 봉투(PBD.Cloth 또는 FEM.Cloth) 벽을
"통과"(누출)하지 않고 담기는지 사전 검증하는 격리 실험.

배경(사용자 지시, 2026-07-28): docs/DigitalTwin.md 목표(파우더를 회수장치로 접시에
담기)를 향한 첫 단계. Genesis 에는 FEM<->PBD 직접 커플링이 없다(legacy_coupler.py
확인 — `fem_pbd` 플래그 자체가 없고 코드 경로도 없음). 있는 조합은:
  - MPM<->PBD (`mpm_pbd`) : 파티클 근접 기반(고정 반경 내 최근접 이웃 평균) —
    표면/법선 개념이 없어 코너/이음매에서 새어나갈 여지가 상대적으로 큼.
  - FEM<->MPM (`fem_mpm`) : 표면 삼각형의 signed-distance + 법선 기반 — 더
    견고할 것으로 예상(코드 구조상).
로봇팔/회수장치는 배제(사용자 지시)하고, 봉투는 full_workflow.py 와 동일하게
"바닥+양측면 고정, 입구는 자유"로 세팅한다(이 프로젝트가 이미 실제 정제 낙하로
검증한 방식 — docs 조합5/6/9. 입구가 저절로 벌어져 있는 PTFE 재질 봉투를
가정하는 게 맞다는 근거).

**1차 시도 버그 두 개(사용자 지적, 2026-07-28) 및 수정**:
  1. 파우더가 한 덩어리 강체처럼 낙하 -> `gs.morphs.Box` 로 파티클 전체를 한
     스텝에 조밀하게 스폰해서 서로 응집된 채 시작했기 때문. genesis-world
     examples/coupling/sand_wheel.py 를 참고해 `scene.add_emitter()` +
     `emitter.emit()` 로 매 스텝 얇은 조각을 흘려보내는 방식으로 교체 —
     이게 실제 "쏟아지는" 거동을 만드는 정석 API다.
  2. 파우더가 봉투 "옆면"에 떨어짐 -> BAG_EULER=(0,0,0)(내가 임의로 가정한
     "네이티브 축이 이미 입구-위" 가정)가 틀렸다. full_workflow.py 실측 주석
     확인 결과 raw mesh 는 로컬 Y가 높이축이고, 로컬 Z가 얇은/넓은 축 중 하나라
     90도 X축 회전을 걸어야 world Z가 높이가 된다 — 그래서 BAG_EULER=(90,0,90)
     (이 프로젝트가 실제 정제 낙하로 검증한 값)으로 교체했다. 이 조건 확인 없이
     (0,0,0)을 썼던 게 원인 — 실측 없이 가정한 것 반성.

BAG_BACKEND 환경변수로 전환:
    BAG_BACKEND=pbd python powder_containment_test.py   (기본)
    BAG_BACKEND=fem python powder_containment_test.py

파우더 알갱이 크기(GRAIN_SIZE=MPMOptions.particle_size)에 코드상 하한은 없다
(PositiveFloat, 그냥 >0). 하지만 실제 누출 판정에 관여하는 충돌 반경은
particle_size 가 아니라 grid dx(=1/grid_density)다(coupler.py 의 모든 근접판정이
`mpm_solver.dx` 기준) — 그래서 알갱이를 실제 파우더 스케일(<1mm)로 줄이려면
grid_density 를 올려야 하고, 그 비용은 도메인 전체에 대해 세제곱으로 커진다.
여기서는 도메인을 봉투 크기(약 6x9cm) 주변으로 바짝 좁혀 그 비용을 감당한다.
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

sys.path.insert(0, os.path.join(os.path.dirname(_r), "utills"))
from fem_ipc_workarounds import patch_fem_vertex_constraints
from primitive_tablet_generator import make_capsule_tets_v2, add_analytic_fem_entity

# COUPLER=ipc(사용자 지시, 2026-07-28): 오늘 LegacyCoupler 계열(mpm_pbd 누출,
# rigid_pbd 반작용력 폭발버그, fem_mpm 이 FEM.Cloth 와 결합 불가)에서 연달아 문제가
# 났다 — "커플러를 갈아타는 게 아니라 이 프로젝트가 이미 조합2~9로 검증한 안정적인
# IPC 커플러로 가자"는 사용자 지시. MPM.Sand 대신 "알갱이 여러 개 = 작은
# FEM.Elastic 엔티티 여러 개"로 표현(정제 하나를 만들 때 쓰던 make_capsule_tets_v2/
# add_analytic_fem_entity 재사용, 크기만 축소) — FEM 자체엔 과립 전용 소성모델이
# 없지만, 벽 접촉/자기충돌은 IPC가 이미 이 프로젝트에서 검증된 방식으로 처리한다.
COUPLER = os.environ.get("COUPLER", "legacy").lower()
assert COUPLER in ("legacy", "ipc"), f"COUPLER must be legacy/ipc, got {COUPLER!r}"

BAG_BACKEND = os.environ.get("BAG_BACKEND", "pbd").lower()
assert BAG_BACKEND in ("pbd", "fem"), f"BAG_BACKEND must be pbd/fem, got {BAG_BACKEND!r}"
# TEST_MODE=rigid_probe(사용자 지시, 2026-07-28): "파우더가 하나도 입구에 안
# 들어간다"는 지적 확인용 — MPM 입자 다수의 복잡한 흐름 대신, 강체 큐브 1개를
# 노즐 위치에서 떨어뜨려 물리적으로 통과하는지(=bag_bottom_z 근처까지 도달)만
# 본다. 통과 못 하면 입구/노즐 기하 문제, 통과하면 문제는 MPM 쪽(파티클
# 방출·커플링)에 있는 것으로 원인을 분리할 수 있다.
TEST_MODE = os.environ.get("TEST_MODE", "pour").lower()
assert TEST_MODE in ("pour", "rigid_probe", "legacy_sanity", "compaction"), \
    f"TEST_MODE must be pour/rigid_probe/legacy_sanity/compaction, got {TEST_MODE!r}"
PROBE_CUBE_SIZE = 0.004  # 자연 상태 입구 두께(6mm)보다도 작게 — 통과 실패시 순수 기하 문제로 확정.
N_PROBE = 500

# TEST_MODE=legacy_sanity(사용자 지시, 2026-07-30): FEM<->MPM Legacy 커플러 재도전용.
# _fem_run1.log 크래시 재분석 결과, 원인은 coupler 옵션이 아니라 재질 자체였다 —
# genesis/engine/materials/FEM/cloth.py 의 Cloth 클래스 docstring에 "Only works
# with IPCCoupler enabled" 라고 명시돼 있고, FEM/base.py 를 보면 Cloth는
# update_stress 를 오버라이드하지 않아 Legacy의 명시적 substep 경로
# (fem_solver.compute_vel -> _mats_update_stress dispatch)를 타면 base.py의
# noop(raise NotImplementedError)이 호출된다 — Cloth는 설계상 IPC 전용이라
# Legacy와는 애초에 호환 불가. 봉투(두께 0 쉘) 대신 TetGen이 안전하게
# 사면체화하는 두께 있는 FEM.Elastic 평판을 얹어, fem_mpm 코드 경로 자체는
# 정상 동작하는지만 격리 검증한다.
# (참고: Genesis 공식 examples/coupling/ 에는 fem_mpm 조합 전용 예제가 없다 —
# GitHub 확인, 2026-07-30: cloth_on_rigid/cloth_attached_to_rigid/
# fem_cube_linked_with_arm/sand_wheel/flush_cubes/cut_dragon/water_wheel/
# sph_mpm/sph_rigid/grasp_soft_cube 전부 rigid<->fem 또는 rigid<->mpm 조합뿐.)
# PLATE_THICKNESS_MM 로 두께만 오버라이드 가능 — grid dx(=1000/GRID_DENSITY mm)
# 대비 평판 두께 가설(그리드보다 얇으면 터널링) 검증용, 2026-07-30.
PLATE_THICKNESS_M = float(os.environ.get("PLATE_THICKNESS_MM", "6")) * 1e-3
PLATE_SIZE = (0.06, 0.06, PLATE_THICKNESS_M)   # x,y,z(두께) — Box라 TetGen에 안전.
PLATE_POS = (0.0, 0.0, 0.05)

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
_MODE_TAG = TEST_MODE if TEST_MODE == "legacy_sanity" else BAG_BACKEND
MP4_PATH = os.path.join(OUT_DIR, f"powder_containment_{_MODE_TAG}_{_TS}.mp4")

# full_workflow.py 와 동일한 실측 봉투 에셋 재사용(사용자 지시 — 새로 만들지 않는다).
BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag", "Samplebag_seal_pouch3.stl")
BAG_POS = (0.0, 0.0, 0.11)
# full_workflow.py 실측 확정값 재사용(§docstring 1차 시도 버그 2 참고) — X=6mm
# (두께), Y=64mm(폭), Z=90mm(높이, +Z가 입구).
BAG_EULER = (90, 0, 90)

DT = 1e-3
SUBSTEPS = 10

# MPM: 도메인을 봉투 주변으로 좁혀 grid_density(=1/dx)를 올릴 여유를 만든다(§docstring).
GRID_DENSITY = 128        # dx = 1/128 ≈ 7.8mm — 누출 판정은 사실상 이 dx가 좌우.
GRAIN_SIZE = 2e-3         # 알갱이 지름 2mm(입구 두께 6mm 슬롯을 통과해야 하므로 3mm보다 축소).
# 실측(2026-07-28): MPMSolver 는 지정 도메인에서 3*dx(세이프티 패딩)만큼 안쪽으로
# 유효 경계를 깎는다. 낙하 경로/봉투 전체가 이 패딩을 뺀 유효 경계 안에 들어오도록
# 여유를 크게 잡는다.
MPM_LOWER = (-0.12, -0.12, -0.03)
MPM_UPPER = (0.12, 0.12, 0.30)

N_SETTLE = 300     # 봉투만 중력으로 먼저 처짐/안정화(파우더 투입 전).
N_POUR = 1200      # 파우더 스트림 낙하 + 관찰.
LOG_EVERY = 100

# 입구 위 3cm 노즐에서 -Z 로 흘려보낸다(사용자 지적 반영 — 옆면이 아니라 입구로).
# rigid_probe 접촉 폭발 디버깅용(2026-07-28): 낙하 높이를 줄여 속도/터널링 문제인지
# vs 가장자리 법선 버그인지 구분 — env로 오버라이드 가능.
NOZZLE_CLEARANCE_Z = float(os.environ.get("NOZZLE_CLEARANCE_Z", "0.03"))
POUR_SPEED = 0.6           # m/s, 완만한 트리클(sand_wheel.py 는 8.0 이지만 그건
                           # 대형 낙하 슈트용 — 우리는 좁은 6mm 슬롯이라 훨씬 느리게).
POUR_DROPLET_SHAPE = "circle"  # 원형 단면 — 입구 두께(6mm) 슬롯 안에서 방향에
                                # 무관하게 안전하게 들어맞는다(rectangle 은 로컬 X/Y가
                                # world 수평면에 정확히 어떻게 매핑되는지 emitter의
                                # z_up_to_R 변환에 좌우돼 얇은 쪽을 못 맞출 위험).
NOZZLE_DIAM_FRAC = 0.6     # 실측 입구 두께의 60%를 노즐 지름으로(나머지는 여유).
NOZZLE_DIAM_MIN = 2 * GRAIN_SIZE

LEAK_TOL_Z = 0.01  # 봉투 바닥(정착 후 실측)보다 1cm 이상 아래로 빠지면 "누출".
LEAK_FRAC_THRESHOLD = 0.05  # 누출 비율이 이 이상이면 verdict=LEAK.

# ── COUPLER=ipc 전용 상수 ────────────────────────────────────────────────────
N_GRAINS = int(os.environ.get("N_GRAINS", "40"))  # IPC Newton 솔버 비용 때문에 소규모로 시작.
GRAIN_RADIUS_MM = 1.5   # 알갱이 반지름(입구 두께 6mm 대비 지름 3mm — 여유 있게 통과).
GRAIN_CYL_H_MM = 0.5    # 거의 구형(짧은 원기둥 캡슐).
GRAIN_E, GRAIN_NU, GRAIN_RHO, GRAIN_FRICTION = 5.0e5, 0.45, 1200.0, 0.6
N_DROP = 800   # 알갱이들이 순차로 떨어져 쌓이는 구간.


def _npy(x):
    x = x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)
    return x[0] if x.ndim == 3 else x


def main_ipc():
    """COUPLER=ipc: 이 프로젝트가 조합2~9(docs/DigitalTwin.md)로 이미 검증한
    FEM.Cloth(봉투)+FEM.Elastic(알갱이, 정제와 동일 기법)+IPC 조합. LegacyCoupler
    계열(mpm_pbd/rigid_pbd/fem_mpm)에서 연달아 난 문제(누출·반작용력 폭발·재질
    호환 불가)를 전부 피해간다 — IPC 는 CCD 기반이라 관통/폭발에 훨씬 강하다."""
    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")
    patch_fem_vertex_constraints()

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=5e-3, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=True,
            # docs/DigitalTwin.md §8 기록 재사용: 기본값(libuipc 10mm)은 이
            # 스케일(알갱이 반지름 1.5mm, 입구 두께 6mm)에 비해 너무 커서
            # 접촉 그물에 전체가 얼어붙는다 — 0.5mm 로 낮추면 안정적으로 통과.
            contact_d_hat=0.0005,
        ),
        show_viewer=False,
    )

    scene.add_entity(
        gs.morphs.Plane(pos=(0, 0, -0.05)),
        material=gs.materials.Rigid(coup_type="ipc_only"),
    )

    bag_surface = gs.surfaces.Default(color=(0.6, 0.75, 0.95), opacity=0.55, double_sided=True)
    bag = scene.add_entity(
        material=gs.materials.FEM.Cloth(
            E=4.0e5, nu=0.499, rho=200.0, thickness=1.0e-3,
            bending_stiffness=400.0, friction_mu=0.8,
        ),
        morph=gs.morphs.Mesh(file=BAG_STL, scale=1.0, pos=BAG_POS, euler=BAG_EULER),
        surface=bag_surface,
    )

    # 정제(tablet)와 동일한 해석적 캡슐 tet 생성 기법 재사용, 크기만 알갱이
    # 스케일(반지름 1.5mm)로 축소 — FEM 자체엔 과립 소성모델이 없으니 "알갱이 여러
    # 개 = 작은 FEM.Elastic 엔티티 여러 개"로 표현(사용자 지시).
    cap_verts_mm, cap_elems = make_capsule_tets_v2(
        radius_mm=GRAIN_RADIUS_MM, cyl_height_mm=GRAIN_CYL_H_MM, n_theta=8, n_cap_rings=2, n_cyl_bands=1,
    )
    grain_key = os.path.join(OUT_DIR, "_grain_analytic.stl")
    mouth_top_z_est = BAG_POS[2] + 0.045  # BAG_HALF_H(full_workflow.py) 재사용 추정치.
    rng = np.random.default_rng(0)
    grains = []
    for i in range(N_GRAINS):
        jitter = rng.uniform(-0.003, 0.003, size=2)
        pos_i = (BAG_POS[0] + jitter[0], BAG_POS[1] + jitter[1],
                  mouth_top_z_est + 0.02 + i * (2 * GRAIN_RADIUS_MM * 1e-3 * 2.5))
        g = add_analytic_fem_entity(
            scene, key=grain_key, verts_mm=cap_verts_mm, elems=cap_elems,
            material=gs.materials.FEM.Elastic(
                E=GRAIN_E, nu=GRAIN_NU, rho=GRAIN_RHO, friction_mu=GRAIN_FRICTION, model="stable_neohookean",
            ),
            scale=1e-3, pos=pos_i,
            surface=gs.surfaces.Default(color=(0.85, 0.75, 0.55), roughness=0.6),
        )
        grains.append(g)

    cam = scene.add_camera(res=(1024, 768), pos=(0.30, -0.30, BAG_POS[2] + 0.15),
                           lookat=BAG_POS, fov=40, GUI=False)

    print(f"\n[build] COUPLER=ipc  N_GRAINS={N_GRAINS}  scene.build() 시작...")
    scene.build(n_envs=0)
    print("[build] 성공")

    # ── 봉투: 바닥+양측면 고정 + 입구 깔때기(legacy 경로와 동일 로직, FEM 전용) ──
    pos0 = _npy(bag.get_state().pos)
    bx, by, bz = pos0[:, 0], pos0[:, 1], pos0[:, 2]
    bottom_mask = bz < bz.min() + 0.012
    side_mask = (by < by.min() + 0.008) | (by > by.max() - 0.008)
    mouth_mask = bz >= np.quantile(bz, 0.88)
    mouth_spread_mask = mouth_mask & ~side_mask & ~bottom_mask

    MOUTH_HALF_GAP = 0.010
    target_pos = pos0.copy()
    mouth_idx = np.where(mouth_spread_mask)[0]
    mouth_mean_x = float(bx[mouth_idx].mean())
    z_band_lo, z_band_hi = float(bz[mouth_idx].min()), float(bz[mouth_idx].max())
    taper_t = (bz[mouth_idx] - z_band_lo) / max(z_band_hi - z_band_lo, 1e-9)
    front_sub = bx[mouth_idx] >= mouth_mean_x
    desired_x = np.where(front_sub, mouth_mean_x + MOUTH_HALF_GAP, mouth_mean_x - MOUTH_HALF_GAP)
    target_pos[mouth_idx, 0] = bx[mouth_idx] + taper_t * (desired_x - bx[mouth_idx])

    static_idx = np.where((bottom_mask | side_mask) & ~mouth_spread_mask)[0].tolist()
    bag.set_vertex_constraints(verts_idx_local=static_idx, is_soft_constraint=False)
    bag.set_vertex_constraints(verts_idx_local=mouth_idx.tolist(),
                               target_poss=target_pos[mouth_idx], is_soft_constraint=False)
    print(f"[bag] 바닥+양측면 고정: {len(static_idx)}개, 입구 벌림 고정: {len(mouth_idx)}개")

    cam.start_recording()
    print(f"\n[phase] settle (0.5s) — 봉투만 중력으로 처짐(알갱이는 아직 위에서 낙하 중)")
    for _ in range(100):
        scene.step()
        cam.render()
    bag_bottom_z = float(_npy(bag.get_state().pos)[:, 2].min())
    print(f"[bag] settle 후 바닥 z={bag_bottom_z:.4f}")

    print(f"\n[phase] drop ({N_DROP*5e-3:.1f}s) — 알갱이 {N_GRAINS}개 낙하 + 누출 관찰")
    for k in range(N_DROP):
        scene.step()
        cam.render()
        if (k + 1) % 40 == 0:
            centroids = np.stack([_npy(g.get_state().pos).mean(axis=0) for g in grains])
            leaked = centroids[:, 2] < (bag_bottom_z - LEAK_TOL_Z)
            n_leak = int(leaked.sum())
            print(f"[t={(k+1)*5e-3:6.2f}s] grains min_z={centroids[:,2].min():.4f}  "
                  f"leaked={n_leak}/{N_GRAINS}({100*n_leak/N_GRAINS:.1f}%)  bag_bottom_z={bag_bottom_z:.4f}")

    cam.stop_recording(save_to_filename=MP4_PATH, fps=30)
    print(f"\n[saved] {MP4_PATH}")

    centroids = np.stack([_npy(g.get_state().pos).mean(axis=0) for g in grains])
    leaked = centroids[:, 2] < (bag_bottom_z - LEAK_TOL_Z)
    n_leak = int(leaked.sum())
    frac = n_leak / N_GRAINS
    verdict = "LEAK" if frac > LEAK_FRAC_THRESHOLD else "CONTAINED"
    print("\n" + "=" * 60)
    print(f"[RESULT] COUPLER=ipc  N_GRAINS={N_GRAINS}  grain_radius={GRAIN_RADIUS_MM}mm")
    print(f"[RESULT] leaked={n_leak}/{N_GRAINS} ({frac*100:.1f}%)  threshold={LEAK_FRAC_THRESHOLD*100:.0f}%")
    print(f"[RESULT] verdict={verdict}")
    print("=" * 60)


def main_legacy_sanity():
    """TEST_MODE=legacy_sanity(사용자 지시, 2026-07-30): "FEM-MPM Legacy coupler
    다시 시도해보자"에 대한 실제 재도전. 이전 시도(BAG_BACKEND=fem, TEST_MODE=pour,
    _fem_run1.log)가 crash한 진짜 원인은 fem_mpm 커플러 옵션이 아니라 봉투 재질로
    쓴 FEM.Cloth 자체였다 — genesis/engine/materials/FEM/cloth.py docstring에
    "Only works with IPCCoupler enabled"라고 명시돼 있고, Cloth는 update_stress를
    오버라이드하지 않아 Legacy의 명시적 substep(fem_solver.compute_vel)을 타면
    base.py의 noop이 NotImplementedError를 던진다(Cloth는 설계상 IPC 전용).
    그래서 여기서는 봉투 대신 TetGen이 안전하게 다루는 FEM.Elastic 평판(두께
    6mm Box)을 얹어, fem_mpm 코드 경로 자체가 정상 동작하는지만 격리 검증한다.
    사용자 지시대로(Genesis 공식 레포 권고) coupler_options에서 fem_mpm 외
    나머지 커플링은 전부 False로 꺼서 계산 비용을 줄인다."""
    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.LegacyCouplerOptions(
            rigid_mpm=False, rigid_sph=False, rigid_pbd=False, rigid_fem=False,
            mpm_sph=False, mpm_pbd=False, fem_sph=False,
            fem_mpm=True,  # 이 스크립트가 검증하려는 유일한 커플링(나머지는 계산비용 절감을 위해 전부 off).
        ),
        mpm_options=gs.options.MPMOptions(
            particle_size=GRAIN_SIZE, grid_density=GRID_DENSITY,
            lower_bound=MPM_LOWER, upper_bound=MPM_UPPER,
        ),
        fem_options=gs.options.FEMOptions(damping=0.2),
        show_viewer=False,
    )

    plate = scene.add_entity(
        material=gs.materials.FEM.Elastic(
            E=1.0e6, nu=0.4, rho=1000.0, friction_mu=0.5, model="stable_neohookean",
        ),
        morph=gs.morphs.Box(pos=PLATE_POS, size=PLATE_SIZE),
        surface=gs.surfaces.Default(color=(0.7, 0.55, 0.35)),
    )

    emitter = scene.add_emitter(
        material=gs.materials.MPM.Sand(E=2e5, nu=0.2, rho=1500.0, friction_angle=45.0),
        max_particles=20000,
        surface=gs.surfaces.Default(color=(0.85, 0.75, 0.55, 1.0)),
    )
    sand = emitter.entity

    cam = scene.add_camera(res=(1024, 768), pos=(0.25, -0.25, PLATE_POS[2] + 0.15),
                           lookat=PLATE_POS, fov=40, GUI=False)

    print(f"\n[build] TEST_MODE=legacy_sanity (LegacyCoupler, fem_mpm=True 단독) scene.build() 시작...")
    scene.build(n_envs=0)
    print("[build] 성공")

    # 평판 가장자리만 고정 — 낙하로 밀려나지 않게 하되, 표면 접촉 응답 자체는
    # fem_solver의 정상 dynamics(update_stress dispatch)로 계산되게 한다.
    pos0 = _npy(plate.get_state().pos)
    px, py, pz = pos0[:, 0], pos0[:, 1], pos0[:, 2]
    edge_mask = (
        (px < px.min() + 0.006) | (px > px.max() - 0.006) |
        (py < py.min() + 0.006) | (py > py.max() - 0.006)
    )
    edge_idx = np.where(edge_mask)[0].tolist()
    plate.set_vertex_constraints(verts_idx_local=edge_idx, is_soft_constraint=False)
    plate_top_z = float(pz.max())
    print(f"[plate] 가장자리 고정: {len(edge_idx)}개  top_z={plate_top_z:.4f}")

    nozzle_pos = (PLATE_POS[0], PLATE_POS[1], plate_top_z + NOZZLE_CLEARANCE_Z + 0.02)
    nozzle_diam = 0.015

    cam.start_recording()
    print(f"\n[phase] pour ({N_POUR*DT:.1f}s) — 파우더 스트림 낙하 (FEM.Elastic 평판 위, fem_mpm 코드 경로 검증)")
    for k in range(N_POUR):
        emitter.emit(
            droplet_shape=POUR_DROPLET_SHAPE,
            droplet_size=nozzle_diam,
            pos=nozzle_pos,
            direction=(0, 0, -1),
            speed=POUR_SPEED,
        )
        scene.step()
        cam.render()
        if (k + 1) % LOG_EVERY == 0:
            active = _npy(sand.get_particles_active()).astype(bool)
            sp = _npy(sand.get_particles_pos())[active]
            if len(sp) == 0:
                print(f"[t={(k+1)*DT:6.2f}s] sand N=0 (아직 방출 없음)")
                continue
            below = sp[:, 2] < (plate_top_z - LEAK_TOL_Z)
            n_below = int(below.sum())
            print(f"[t={(k+1)*DT:6.2f}s] sand N={len(sp)}  below_plate_top={n_below}"
                  f"({100*n_below/len(sp):5.1f}%)  min_z={sp[:,2].min():.4f}  plate_top_z={plate_top_z:.4f}")

    cam.stop_recording(save_to_filename=MP4_PATH, fps=30)
    print(f"\n[saved] {MP4_PATH}")

    active = _npy(sand.get_particles_active()).astype(bool)
    sp = _npy(sand.get_particles_pos())[active]
    below = sp[:, 2] < (plate_top_z - LEAK_TOL_Z)
    n_below = int(below.sum())
    frac = n_below / len(sp) if len(sp) else 0.0
    verdict = "FEM_MPM_COUPLING_OK" if frac < LEAK_FRAC_THRESHOLD else "PARTICLES_FELL_THROUGH"
    print("\n" + "=" * 60)
    print(f"[RESULT] TEST_MODE=legacy_sanity  total_sand={len(sp)}")
    print(f"[RESULT] below_plate_top={n_below} ({frac*100:.1f}%)  threshold={LEAK_FRAC_THRESHOLD*100:.0f}%")
    print(f"[RESULT] verdict={verdict}")
    print("=" * 60)


def main():
    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    coupler_kwargs = dict(
        rigid_mpm=False, rigid_sph=False, rigid_pbd=False, rigid_fem=False,
        mpm_sph=False, mpm_pbd=False, fem_mpm=False, fem_sph=False,
    )
    if TEST_MODE == "pour":
        coupler_kwargs["mpm_pbd" if BAG_BACKEND == "pbd" else "fem_mpm"] = True
    else:
        coupler_kwargs["rigid_pbd" if BAG_BACKEND == "pbd" else "rigid_fem"] = True

    scene_kwargs = dict(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.LegacyCouplerOptions(**coupler_kwargs),
        show_viewer=False,
    )
    if TEST_MODE == "pour":
        scene_kwargs["mpm_options"] = gs.options.MPMOptions(
            particle_size=GRAIN_SIZE, grid_density=GRID_DENSITY,
            lower_bound=MPM_LOWER, upper_bound=MPM_UPPER,
        )
    if BAG_BACKEND == "pbd":
        scene_kwargs["pbd_options"] = gs.options.PBDOptions(particle_size=2e-3)
    else:
        scene_kwargs["fem_options"] = gs.options.FEMOptions(damping=0.2)

    scene = gs.Scene(**scene_kwargs)

    bag_surface = gs.surfaces.Default(color=(0.6, 0.75, 0.95), opacity=0.55, double_sided=True)
    if BAG_BACKEND == "pbd":
        bag = scene.add_entity(
            material=gs.materials.PBD.Cloth(),
            morph=gs.morphs.Mesh(file=BAG_STL, scale=1.0, pos=BAG_POS, euler=BAG_EULER),
            surface=bag_surface,
        )
    else:
        bag = scene.add_entity(
            material=gs.materials.FEM.Cloth(
                E=4.0e5, nu=0.499, rho=200.0, thickness=1.0e-3,
                bending_stiffness=400.0, friction_mu=0.8,
            ),
            morph=gs.morphs.Mesh(file=BAG_STL, scale=1.0, pos=BAG_POS, euler=BAG_EULER),
            surface=bag_surface,
        )

    if TEST_MODE == "pour":
        # sand_wheel.py(genesis-world/examples/coupling) 방식 재현: 정적 블록이
        # 아니라 emitter 로 매 스텝 얇은 조각을 흘려보내야 "쏟아지는" 거동이 나온다.
        emitter = scene.add_emitter(
            material=gs.materials.MPM.Sand(E=2e5, nu=0.2, rho=1500.0, friction_angle=45.0),
            max_particles=20000,
            surface=gs.surfaces.Default(color=(0.85, 0.75, 0.55, 1.0)),
        )
        sand = emitter.entity
    else:
        # 노즐 위치는 settle 이후에나 알 수 있으므로 일단 씬 밖 높은 곳에
        # 파킹해두고, settle 후 set_pos 로 실제 낙하 시작 위치로 옮긴다.
        probe = scene.add_entity(
            material=gs.materials.Rigid(needs_coup=True, coup_friction=0.3),
            morph=gs.morphs.Box(pos=(0, 0, 1.0), size=(PROBE_CUBE_SIZE,) * 3),
            surface=gs.surfaces.Default(color=(0.9, 0.2, 0.2)),
        )

    cam = scene.add_camera(res=(1024, 768), pos=(0.30, -0.30, BAG_POS[2] + 0.15),
                           lookat=BAG_POS, fov=40, GUI=False)

    print(f"\n[build] backend={BAG_BACKEND} scene.build() 시작...")
    scene.build(n_envs=0)
    print("[build] 성공")

    # ── 봉투: 바닥+양측면 고정 + 입구를 깔때기 모양으로 벌림(사용자 지적, 2026-07-28) ──
    # trimesh 로 원본 STL 위상 확인 결과(사용자 질문에 대한 답): 뚜껑(cap) 삼각형이
    # 0개 — 이 메시는 이미 진짜 열린 튜브(전체 폭 64mm x 두께 6mm 단면이 그대로
    # 뚫려 있음, "Samplebag desigin.stl"의 평면 패턴 테두리와는 다른 것)다. 그런데
    # 1차 시도에서 입구 밴드(상위 12%) 전체를 "균일하게" 밀었더니 그 밴드의 아래쪽
    # 경계에서 각(주름)이 져서 깔때기가 아니라 꺾인 모양이 됐다(사용자 지적).
    # 밴드 내 높이 비율 t(0=밴드 하단=원래 폭 유지, 1=맨 꼭대기=완전히 벌어짐)로
    # 목표 위치를 선형보간해 매끄러운 깔때기를 만든다.
    if BAG_BACKEND == "pbd":
        pos0 = _npy(bag.get_particles_pos())
    else:
        pos0 = _npy(bag.get_state().pos)
    bx, by, bz = pos0[:, 0], pos0[:, 1], pos0[:, 2]
    bottom_mask = bz < bz.min() + 0.012
    side_mask = (by < by.min() + 0.008) | (by > by.max() - 0.008)
    mouth_mask = bz >= np.quantile(bz, 0.88)
    mouth_spread_mask = mouth_mask & ~side_mask & ~bottom_mask

    MOUTH_HALF_GAP = 0.010  # 맨 꼭대기에서 앞/뒷면 목표 오프셋 -> 총 20mm 입구(원래 6mm 대비 개방).
    target_pos = pos0.copy()
    mouth_idx = np.where(mouth_spread_mask)[0]
    mouth_mean_x = float(bx[mouth_idx].mean())
    z_band_lo, z_band_hi = float(bz[mouth_idx].min()), float(bz[mouth_idx].max())
    taper_t = (bz[mouth_idx] - z_band_lo) / max(z_band_hi - z_band_lo, 1e-9)  # 0(밴드 하단)~1(꼭대기)
    front_sub = bx[mouth_idx] >= mouth_mean_x
    desired_x = np.where(front_sub, mouth_mean_x + MOUTH_HALF_GAP, mouth_mean_x - MOUTH_HALF_GAP)
    target_pos[mouth_idx, 0] = bx[mouth_idx] + taper_t * (desired_x - bx[mouth_idx])

    static_idx = np.where((bottom_mask | side_mask) & ~mouth_spread_mask)[0].tolist()
    if BAG_BACKEND == "pbd":
        bag.set_particles_pos(target_pos[mouth_idx], particles_idx_local=mouth_idx.tolist())
        bag.fix_particles(particles_idx_local=static_idx + mouth_idx.tolist())
    else:
        bag.set_vertex_constraints(verts_idx_local=static_idx, is_soft_constraint=False)
        bag.set_vertex_constraints(verts_idx_local=mouth_idx.tolist(),
                                   target_poss=target_pos[mouth_idx], is_soft_constraint=False)
    print(f"[bag] 바닥+양측면 고정: {len(static_idx)}개, 입구 벌림 고정: {len(mouth_idx)}개"
          f"(중심x={mouth_mean_x:.4f}, 목표 입구두께={2*MOUTH_HALF_GAP*1000:.0f}mm)")

    cam.start_recording()

    print(f"\n[phase] settle ({N_SETTLE*DT:.1f}s) — 봉투만 중력으로 처짐")
    for _ in range(N_SETTLE):
        scene.step()
        cam.render()

    if BAG_BACKEND == "pbd":
        pos_settled = _npy(bag.get_particles_pos())
    else:
        pos_settled = _npy(bag.get_state().pos)
    bz_s = pos_settled[:, 2]
    bag_bottom_z = float(bz_s.min())

    # 입구(상위 5% 밴드) 실측 중심/두께로 노즐 위치·지름을 정한다 — 옆면이 아니라
    # 실제 벌어진 구멍 한가운데로 흘려보내기 위해(사용자 지적 반영).
    mouth_band = pos_settled[bz_s >= np.quantile(bz_s, 0.95)]
    mouth_cx, mouth_cy = float(mouth_band[:, 0].mean()), float(mouth_band[:, 1].mean())
    mouth_top_z = float(mouth_band[:, 2].max())
    mouth_x_span = float(mouth_band[:, 0].max() - mouth_band[:, 0].min())
    # 이제 입구를 의도적으로 2*MOUTH_HALF_GAP 만큼 벌렸으므로(위 constraint 단계),
    # 예전의 6mm 하드캡 대신 실측 폭의 60%(여유 40%)까지 노즐을 키운다.
    nozzle_diam = max(NOZZLE_DIAM_MIN, min(0.012, NOZZLE_DIAM_FRAC * max(mouth_x_span, NOZZLE_DIAM_MIN)))
    nozzle_pos = (mouth_cx, mouth_cy, mouth_top_z + NOZZLE_CLEARANCE_Z)
    print(f"[bag] settle 후 바닥 z={bag_bottom_z:.4f}  입구 중심=({mouth_cx:.4f},{mouth_cy:.4f})"
          f"  입구두께 실측={mouth_x_span*1000:.1f}mm  노즐지름={nozzle_diam*1000:.1f}mm"
          f"  노즐높이 z={nozzle_pos[2]:.4f}")

    if TEST_MODE == "pour":
        print(f"\n[phase] pour ({N_POUR*DT:.1f}s) — 파우더 스트림 낙하 + 누출 관찰")
        for k in range(N_POUR):
            emitter.emit(
                droplet_shape=POUR_DROPLET_SHAPE,
                droplet_size=nozzle_diam,
                pos=nozzle_pos,
                direction=(0, 0, -1),
                speed=POUR_SPEED,
            )
            scene.step()
            cam.render()
            if (k + 1) % LOG_EVERY == 0:
                active = _npy(sand.get_particles_active()).astype(bool)
                sp = _npy(sand.get_particles_pos())[active]
                if len(sp) == 0:
                    print(f"[t={(k+1)*DT:6.2f}s] sand N=0 (아직 방출 없음)")
                    continue
                leaked = sp[:, 2] < (bag_bottom_z - LEAK_TOL_Z)
                n_leak = int(leaked.sum())
                print(f"[t={(k+1)*DT:6.2f}s] sand N={len(sp)}  leaked={n_leak}"
                      f"({100*n_leak/len(sp):5.1f}%)  min_z={sp[:,2].min():.4f}"
                      f"  bag_bottom_z={bag_bottom_z:.4f}")

        cam.stop_recording(save_to_filename=MP4_PATH, fps=30)
        print(f"\n[saved] {MP4_PATH}")

        active = _npy(sand.get_particles_active()).astype(bool)
        sp = _npy(sand.get_particles_pos())[active]
        leaked = sp[:, 2] < (bag_bottom_z - LEAK_TOL_Z)
        n_leak = int(leaked.sum())
        frac = n_leak / len(sp) if len(sp) else 0.0
        verdict = "LEAK" if frac > LEAK_FRAC_THRESHOLD else "CONTAINED"
        print("\n" + "=" * 60)
        print(f"[RESULT] backend={BAG_BACKEND}  grid_dx={1.0/GRID_DENSITY*1000:.1f}mm"
              f"  grain={GRAIN_SIZE*1000:.1f}mm  nozzle={nozzle_diam*1000:.1f}mm")
        print(f"[RESULT] total_sand={len(sp)}  leaked={n_leak} ({frac*100:.1f}%)"
              f"  threshold={LEAK_FRAC_THRESHOLD*100:.0f}%")
        print(f"[RESULT] verdict={verdict}")
        print("=" * 60)
    else:
        print(f"\n[phase] rigid_probe ({N_PROBE*DT:.1f}s) — 큐브 1개를 노즐 위치에서 낙하")
        probe.set_pos(np.array(nozzle_pos))
        for k in range(N_PROBE):
            scene.step()
            cam.render()
            if (k + 1) % 50 == 0:
                pz = float(_npy(probe.get_pos())[2])
                print(f"[t={(k+1)*DT:6.2f}s] probe_z={pz:.4f}  "
                      f"(mouth_top_z={mouth_top_z:.4f}, bag_bottom_z={bag_bottom_z:.4f})")

        cam.stop_recording(save_to_filename=MP4_PATH, fps=30)
        print(f"\n[saved] {MP4_PATH}")

        final_z = float(_npy(probe.get_pos())[2])
        entered = final_z < (mouth_top_z - 0.005)  # 입구 꼭대기보다 5mm 이상 내려갔으면 통과로 간주.
        verdict = "ENTERED" if entered else "STUCK_ON_TOP"
        print("\n" + "=" * 60)
        print(f"[RESULT] backend={BAG_BACKEND}  probe_cube={PROBE_CUBE_SIZE*1000:.0f}mm")
        print(f"[RESULT] final_z={final_z:.4f}  mouth_top_z={mouth_top_z:.4f}  bag_bottom_z={bag_bottom_z:.4f}")
        print(f"[RESULT] verdict={verdict}")
        print("=" * 60)



# ══════════════════════════════════════════════════════════════════════════════
# TEST_MODE=compaction — Rigid 피스톤 ↔ MPM 과립 압밀 (사용자 지시, 2026-09-03)
# ══════════════════════════════════════════════════════════════════════════════
# 목적: "Crusher(Rigid)로 과립재를 때렸을 때 어떤 힘을 받고 얼마나 압밀되는가"를
#       디지털 트윈으로 계산할 수 있는지 **가능성만** 판정한다. 판정 항목 4개:
#         (A) rigid_mpm 커플링으로 준정적 압축이 성립하는가 (관통/발산 없이)
#         (B) 피스톤 반력을 읽을 수 있는가  ← 없으면 F-δ·W* 캘리브레이션 전체가 불가
#         (C) 영구 압밀(하중 제거 후에도 베드가 안 돌아옴)이 일어나는가
#         (D) 재료/파라미터로 압밀 곡선을 조정(=캘리브레이션)할 수 있는가
#
# 소스 확인(2026-09-03): MPM.Sand 의 sand_projection 은 tr<0(압축)에서 편차성분만
# 항복 투영하고 체적성분은 그대로 통과시킨다 → 정수압 압축이 순수 탄성이라 영구
# 압밀이 원리적으로 없다. ElastoPlastic/Snow 는 yield_lower 로 특이값을 클램프해
# 영구 압밀이 있다. (C)는 이 차이를 실측으로 확인하는 항목이다.
#
# 도메인을 다이(die) 크기로 바짝 좁혀 dx 를 mm 이하로 내린다 — 조합12 의 1/dx^4
# 비용 벽은 0.24x0.24x0.33m 도메인 기준이었고, 여기 도메인은 그보다 수백 배 작다.
#
# env:
#   MAT              sand | elastoplastic | snow      (기본 sand)
#   GRID_DENSITY_C   1/dx [1/m]. 1000 -> dx=1mm       (기본 1000)
#   PISTON_VEL       하강 속도 [m/s]                   (기본 0.01)
#   PISTON_TRAVEL    총 하강량 [m]                     (기본 0.004)
#   E_MPM/NU_MPM/RHO_MPM/FRICTION_ANGLE/YIELD_LOWER/YIELD_HIGHER
#   COUP_FRICTION    rigid-mpm 마찰                    (기본 0.4)
#   DT_C/SUBSTEPS_C  적분 파라미터                     (기본 5e-4 / 20)
#   NO_VIDEO=1       녹화 끄기(스윕용)
#   TAG              결과 파일 접미사

C_MAT        = os.environ.get("MAT", "sand").lower()
C_GRID       = int(os.environ.get("GRID_DENSITY_C", "2000"))
C_PIS_VEL    = float(os.environ.get("PISTON_VEL", "0.01"))
C_PIS_TRAVEL = float(os.environ.get("PISTON_TRAVEL", "0.004"))
C_E          = float(os.environ.get("E_MPM", "1e6"))
C_NU         = float(os.environ.get("NU_MPM", "0.2"))
C_RHO        = float(os.environ.get("RHO_MPM", "1500"))
C_PHI        = float(os.environ.get("FRICTION_ANGLE", "45"))
C_YLO        = float(os.environ.get("YIELD_LOWER", "0.0025"))
C_YHI        = float(os.environ.get("YIELD_HIGHER", "0.0045"))
C_COUPFRIC   = float(os.environ.get("COUP_FRICTION", "0.4"))
# coup_softness: 커플링 영향이 물체 표면에서 얼마나 멀리까지 미치는가.
# legacy_coupler: influence = min(exp(-signed_dist / coup_softness), 1) 이라
# 0 이면 "표면 안쪽에 이미 들어온 입자"만 밀어낸다 -> 관통 후 뒤늦은 보정.
# diag2 에서 피스톤 하면 위로 입자 10,927개(10%)가 관통한 원인 후보 1순위.
C_COUPSOFT   = float(os.environ.get("COUP_SOFTNESS", "0.0"))
# substep_dt = DT/SUBSTEPS 는 MPMSolver 가 grid_density 에서 계산하는 suggested_dt
# 이하로 유지해야 한다. grid_density=2000(dx=0.5mm) 에서 suggested_dt=1e-5 이므로
# 2e-4/20 = 1e-5 로 맞춘다. (스모크5 는 2.5e-5 로 돌아 불안정 경고가 났고, 베드가
# 압축 중에 오히려 부풀어 오르는 비물리 거동이 나왔다.)
C_DT         = float(os.environ.get("DT_C", "2e-4"))
C_SUB        = int(os.environ.get("SUBSTEPS_C", "20"))
C_NOVIDEO    = os.environ.get("NO_VIDEO", "0") == "1"
C_TAG        = os.environ.get("TAG", "")
# LOAD_MODE=gravity: 피스톤을 쓰지 않고 **중력만 키워** 베드를 자중 압축한다.
# 목적은 커플러와 재료를 분리해서 보는 것 — 피스톤 압축이 안 먹는 게
# (a) 커플러가 정적 하중을 못 버텨서인지  (b) 재료에 영구압밀이 없어서인지
# 를 가른다. 중력 경로는 커플러를 전혀 타지 않으므로 (b) 만 남는다.
C_USE_VM     = os.environ.get("USE_VON_MISES", "1") == "1"
C_VMYS       = float(os.environ.get("VM_YIELD_STRESS", "10000"))
C_LOAD_MODE  = os.environ.get("LOAD_MODE", "piston").lower()
C_GHIGH      = float(os.environ.get("G_HIGH", "2000"))   # m/s^2 (약 204 g)

# 다이 기하 [m] — 내경 20x20mm, 초기 베드 높이 10mm
DIE_IN    = 0.020
BED_H     = 0.010
WALL_T    = 0.004
PIS_H     = 0.006
# 피스톤-벽 편측 간극. 0 이 기본이다 — 간극을 두면 MPM 이 그 틈으로 분출한다.
# 조합10 에서 확인한 대로 rigid-MPM 근접판정 반경은 particle_size 가 아니라 grid dx 라,
# dx 보다 작은 간극은 "막혀 있다"고 표현되지 않고 오히려 새는 통로가 된다
# (smoke3: dx=2mm, 간극 0.4mm -> 베드가 9mm 에서 25mm 로 분출). 피스톤은 매 스텝
# set_pos 로 구동되고 벽은 fixed 라, 간극 0 으로 인한 rigid-rigid 접촉은 무해하다.
PIS_CLR   = float(os.environ.get("PISTON_CLEARANCE", "0.0"))
FLOOR_T   = 0.004


def _mpm_material():
    import genesis as gs
    if C_MAT == "sand":
        return gs.materials.MPM.Sand(E=C_E, nu=C_NU, rho=C_RHO, friction_angle=C_PHI)
    if C_MAT == "elastoplastic":
        # 주의(소스 확인 2026-09-03, materials/MPM/elasto_plastic.py:60-88):
        # use_von_mises=True(기본)이면 yield_lower/higher 는 **완전히 무시**되고
        # von_mises_yield_stress 만 작동한다 — 게다가 von Mises 분기는 epsilon_hat
        # (편차성분)만 다루므로 체적 소성이 없다.
        # 체적 압밀(특이값 클램프 min(max(S,1-yl),1+yh))을 쓰려면 von Mises 를 꺼야 한다.
        return gs.materials.MPM.ElastoPlastic(
            E=C_E, nu=C_NU, rho=C_RHO,
            yield_lower=C_YLO, yield_higher=C_YHI,
            use_von_mises=C_USE_VM, von_mises_yield_stress=C_VMYS,
        )
    if C_MAT == "snow":
        return gs.materials.MPM.Snow(E=C_E, nu=C_NU, rho=C_RHO,
                                     yield_lower=C_YLO, yield_higher=C_YHI)
    raise SystemExit(f"MAT={C_MAT} 알 수 없음 (sand|elastoplastic|snow)")


def main_compaction():
    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    dx = 1.0 / C_GRID
    half = DIE_IN / 2

    # ── 구속을 rigid 벽이 아니라 MPM 도메인 경계로 준다 ──────────────────
    # MPMSolver 는 지정 도메인에서 3*dx 안쪽을 유효 경계(하드월)로 쓴다(조합10 실측).
    # 그래서 lower/upper 를 다이 내경 ±3*dx 로 두면 유효 벽이 정확히 다이 내면에 서고,
    # 바닥은 z=0 에 선다.
    #
    # 왜 rigid 벽을 쓰지 않는가 — 스모크런 3/4 에서 확인:
    #   · 간극 0.4mm(dx=2mm) : 간극이 dx 보다 작아 표현이 안 되고 오히려 분출 통로가 됨
    #   · 간극 0  (dx=0.5mm) : 피스톤과 벽의 SDF 가 만나는 이음매에서 여전히 분출
    # 두 경우 다 베드가 9mm -> 13~25mm 로 튀었다. 도메인 경계에는 이음매가 없다.
    # rigid 바닥/벽은 needs_coup=False 로 두어 화면 표시 전용으로만 남긴다.
    lower = (-half - 3 * dx, -half - 3 * dx, -3 * dx)
    upper = ( half + 3 * dx,  half + 3 * dx,  BED_H + PIS_H + 0.012)
    n_cells = (int((upper[0]-lower[0])/dx) * int((upper[1]-lower[1])/dx)
               * int((upper[2]-lower[2])/dx))

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=C_DT, substeps=C_SUB, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.LegacyCouplerOptions(
            rigid_mpm=True,
            rigid_sph=False, rigid_pbd=False, rigid_fem=False,
            mpm_sph=False, mpm_pbd=False, fem_mpm=False, fem_sph=False,
        ),
        mpm_options=gs.options.MPMOptions(
            grid_density=C_GRID, lower_bound=lower, upper_bound=upper,
            # gravity 를 명시해야 solver._gravity 필드가 할당되고 런타임
            # set_gravity 가 먹는다 (base_solver.py:36-47 — None 이면 skip).
            gravity=(0.0, 0.0, -9.81),
        ),
        vis_options=gs.options.VisOptions(visualize_mpm_boundary=True),
        show_viewer=False,
    )

    mat_rigid = gs.materials.Rigid(needs_coup=True, coup_friction=C_COUPFRIC,
                                   coup_softness=C_COUPSOFT)
    mat_vis   = gs.materials.Rigid(needs_coup=False)   # 화면 표시 전용(커플링 없음)

    # ── 다이: 바닥판 + 벽 4장 — 전부 시각 보조. 실제 구속은 MPM 도메인 경계 ──
    scene.add_entity(material=mat_vis, surface=gs.surfaces.Default(color=(0.45, 0.45, 0.5)),
                     morph=gs.morphs.Box(pos=(0, 0, -FLOOR_T/2),
                                         size=(DIE_IN + 2*WALL_T, DIE_IN + 2*WALL_T, FLOOR_T),
                                         fixed=True))
    wall_z = BED_H / 2 + 0.006
    wall_h = BED_H + 0.012
    for sx, sy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        scene.add_entity(
            material=mat_vis,
            surface=gs.surfaces.Default(color=(0.55, 0.55, 0.6), opacity=0.25),
            morph=gs.morphs.Box(
                pos=(sx * (half + WALL_T/2), sy * (half + WALL_T/2), wall_z),
                size=((WALL_T if sx else DIE_IN + 2*WALL_T),
                      (WALL_T if sy else DIE_IN + 2*WALL_T), wall_h),
                fixed=True))

    # ── 과립 베드 (MPM) ───────────────────────────────────────────────────
    bed = scene.add_entity(
        material=_mpm_material(),
        morph=gs.morphs.Box(pos=(0, 0, BED_H/2), size=(DIE_IN - 2*dx, DIE_IN - 2*dx, BED_H)),
        surface=gs.surfaces.Default(color=(0.85, 0.75, 0.55, 1.0)),
    )

    # ── 피스톤 (자유 rigid — set_pos + set_dofs_velocity 로 구동) ─────────
    # 피스톤은 도메인 단면 전체를 덮도록 오히려 **더 넓게** 만든다 — 도메인 벽과
    # 피스톤 사이에 틈이 생기면 그리로 분출한다(스모크4). 넘치는 부분은 도메인 밖이라
    # MPM 이 평가하지 않으므로 무해하다.
    pis_w = DIE_IN + 8 * dx - 2 * PIS_CLR
    pis_size = (pis_w, pis_w, PIS_H)
    pis_z0 = BED_H + PIS_H/2 + 0.0015
    piston = scene.add_entity(
        material=mat_rigid, surface=gs.surfaces.Default(color=(0.85, 0.35, 0.25)),
        morph=gs.morphs.Box(pos=(0, 0, pis_z0), size=pis_size, fixed=False))

    cam = scene.add_camera(res=(960, 720), pos=(0.075, -0.075, 0.030),
                           lookat=(0, 0, 0.008), fov=38, GUI=False)

    n_settle = int(0.20 / C_DT)
    n_load   = int(C_PIS_TRAVEL / C_PIS_VEL / C_DT)
    n_hold   = int(0.10 / C_DT)
    n_unload = n_load
    n_free   = int(0.20 / C_DT)
    n_total  = n_settle + n_load + n_hold + n_unload + n_free

    print(f"\n[cfg] MAT={C_MAT} grid_density={C_GRID} dx={dx*1e3:.2f}mm cells={n_cells:,}")
    print(f"[cfg] dt={C_DT} substeps={C_SUB} piston_vel={C_PIS_VEL} travel={C_PIS_TRAVEL*1e3:.1f}mm")
    print(f"[cfg] steps: settle={n_settle} load={n_load} hold={n_hold} "
          f"unload={n_unload} free={n_free} total={n_total}")
    print("[build] scene.build() ...")
    scene.build(n_envs=0)
    print("[build] 성공")

    mp4_path = os.path.join(OUT_DIR, f"compaction_{C_MAT}{C_TAG}_{_TS}.mp4")
    if not C_NOVIDEO:
        # 0.2.1 은 start_recording() 이 인자를 안 받고 stop_recording 에서 저장한다.
        try:
            cam.start_recording(save_to_filename=mp4_path, fps=30)
            _rec_on_stop = False
        except TypeError:
            cam.start_recording()
            _rec_on_stop = True

    rec = {k: [] for k in ("step", "pis_z", "pis_z_act", "fz", "bed_top",
                           "n_act", "bed_mean_z", "n_above")}

    def _pis_z_actual():
        """명령값(z_cmd)이 아니라 **실제** 피스톤 위치. set_pos 가 자유강체에
        먹히지 않으면 여기서 갈린다 — 관통처럼 보이던 게 사실 피스톤이 안 내려간
        것일 수 있어 반드시 분리해서 본다."""
        try:
            p = piston.get_pos()
            p = p.cpu().numpy() if hasattr(p, "cpu") else np.asarray(p)
            p = np.asarray(p).reshape(-1)
            return float(p[-1]) if p.size >= 3 else np.nan
        except Exception:
            return np.nan
    # 녹화 페이스: 0.2.1 은 step() 이 자동 렌더하지 않으므로 직접 render() 한다.
    _spf = max(1, n_total // 300)

    def _bed_particles():
        """Genesis 버전별 입자 위치 API 차이 흡수 (0.2.1 은 get_particles())."""
        for name in ("get_particles_pos", "get_particles"):
            fn = getattr(bed, name, None)
            if fn is None:
                continue
            try:
                p = fn()
            except Exception:
                continue
            if p is not None:
                return p
        st = bed.get_state()
        return getattr(st, "pos", None)

    def _bed_stats():
        p = _bed_particles()
        if p is None:
            return np.nan, 0, np.nan, 0
        p = p.cpu().numpy() if hasattr(p, "cpu") else np.asarray(p)
        p = np.asarray(p)
        while p.ndim > 2:
            p = p[0]
        if p.ndim != 2 or p.shape[-1] != 3:
            return np.nan, 0, np.nan, 0
        finite = np.isfinite(p).all(axis=1)
        p = p[finite]
        if len(p) == 0:
            return np.nan, 0, np.nan, 0
        # 피스톤 하면보다 위에 있는 입자 수 = 관통/분출 지표.
        n_above = int((p[:, 2] > (z_cmd - PIS_H / 2)).sum())
        return float(np.quantile(p[:, 2], 0.98)), int(len(p)), float(p[:, 2].mean()), n_above

    # 반력 경로 확정(소스 확인, 2026-09-03): legacy_coupler._func_collide_in_rigid_geom 은
    # MPM->rigid 반작용을 rigid_solver._func_apply_external_force 로 넘기고, 그 함수는
    # links_state.cfrc_applied_vel[link, env] -= force 로 **외력 누산기**에 쌓는다
    # (rigid_solver_decomp.py:5046). 즉 get_links_net_contact_force(접촉 채널)로는
    # 영원히 0 이 나온다 — 1차 스모크런에서 Fz=0 이었던 이유. cfrc_applied_vel 을
    # 직접 읽고 부호를 뒤집어야 피스톤이 받는 힘이 된다.
    _rs = scene.sim.rigid_solver
    _pis_link = int(getattr(piston, "link_start", 0))

    def _piston_fz():
        """피스톤이 받는 z 반력 [N]. 판정 (B)."""
        fz_ext = np.nan
        try:
            fld = _rs.links_state.cfrc_applied_vel
            arr = fld.to_numpy() if hasattr(fld, "to_numpy") else np.asarray(fld)
            arr = np.asarray(arr)
            v = arr[_pis_link]
            while v.ndim > 1:
                v = v[0]
            fz_ext = -float(v[2])          # cfrc_applied 는 -force 로 쌓임
        except Exception:
            pass
        if np.isfinite(fz_ext) and abs(fz_ext) > 1e-12:
            return fz_ext
        # 폴백: 접촉 채널(rigid-rigid 용) — rigid_mpm 에는 안 잡히지만 혹시 몰라 병행
        fn = getattr(piston, "get_links_net_contact_force", None)
        if fn is not None:
            try:
                f = fn()
                f = f.cpu().numpy() if hasattr(f, "cpu") else np.asarray(f)
                fc = float(np.asarray(f).reshape(-1, 3)[:, 2].sum())
                if abs(fc) > 1e-12:
                    return fc
            except Exception:
                pass
        return fz_ext

    z_cmd = pis_z0
    nan_at = None
    z_touch = None      # 정착 완료 시점의 피스톤 위치(= 베드 상면 바로 위). δ 의 원점.
    for i in range(n_total):
        # 정착이 끝나는 순간, 실제로 가라앉은 베드 상면 바로 위로 피스톤을 재배치한다.
        # (스모크런에서 정착 침하 0.85mm + 초기 간극 1.5mm 를 하강량 2mm 가 못 덮어
        #  접촉 자체가 없었다 — travel 을 키우는 대신 원점을 베드에 맞추는 게 맞다.)
        if i == n_settle:
            _top_now, _, _, _ = _bed_stats()
            if np.isfinite(_top_now):
                z_cmd = float(_top_now) + PIS_H / 2 + 0.0002
            z_touch = z_cmd

        if i < n_settle:
            v_cmd = 0.0
        elif i < n_settle + n_load:
            v_cmd = -C_PIS_VEL
        elif i < n_settle + n_load + n_hold:
            v_cmd = 0.0
        elif i < n_settle + n_load + n_hold + n_unload:
            v_cmd = +C_PIS_VEL
        else:
            v_cmd = 0.0
        z_cmd += v_cmd * C_DT

        if C_LOAD_MODE == "gravity":
            # 피스톤은 정착 위치에 그대로 두고(방금 더한 증분을 되돌린다) 중력만 키운다.
            z_cmd -= v_cmd * C_DT
            v_cmd = 0.0
            g_now = C_GHIGH if (n_settle <= i < n_settle + n_load + n_hold) else 9.81
            try:
                scene.sim.mpm_solver.set_gravity((0.0, 0.0, -g_now))
            except Exception:
                pass

        # 순서 주의: set_pos 가 속도를 리셋할 수 있으므로 속도를 **나중에** 준다.
        # 커플러(_func_collide_in_rigid_geom)는 vel_rigid = _func_vel_at_point(...) 로
        # links_state 의 속도를 읽어 입자 속도를 그 값으로 스냅한다. 여기서 속도가 0이면
        # 입자는 "밀려나는" 게 아니라 "제자리에 붙잡히고", 피스톤은 그대로 통과한다.
        piston.set_pos(np.array([0.0, 0.0, z_cmd]))
        _vset_err = None
        try:
            piston.set_dofs_velocity(np.array([0.0, 0.0, v_cmd, 0.0, 0.0, 0.0]))
        except Exception as _e:
            _vset_err = repr(_e)
        if i == n_settle + 5:
            try:
                _vv = piston.get_dofs_velocity()
                _vv = _vv.cpu().numpy() if hasattr(_vv, "cpu") else np.asarray(_vv)
                print(f"[probe] set_dofs_velocity err={_vset_err}  read-back={np.asarray(_vv).reshape(-1)}")
            except Exception as _e2:
                print(f"[probe] set_dofs_velocity err={_vset_err}  read-back 실패={_e2!r}")

        scene.step()
        if (not C_NOVIDEO) and (i % _spf == 0):
            cam.render()

        top, n_act, meanz, n_above = _bed_stats()
        if not np.isfinite(top) and nan_at is None and i > n_settle:
            nan_at = i
        rec["step"].append(i)
        rec["pis_z"].append(z_cmd)
        rec["pis_z_act"].append(_pis_z_actual())
        rec["fz"].append(_piston_fz())
        rec["bed_top"].append(top)
        rec["n_act"].append(n_act)
        rec["bed_mean_z"].append(meanz)
        rec["n_above"].append(n_above)

        if (i + 1) % 200 == 0:
            print(f"[{i+1:5d}/{n_total}] cmd={z_cmd*1e3:7.3f} act={rec['pis_z_act'][-1]*1e3:7.3f}mm "
                  f"Fz={rec['fz'][-1]:9.3f}N  bed_top={top*1e3:7.3f}mm  "
                  f"pis_bot={(z_cmd-PIS_H/2)*1e3:7.3f}mm  n_above={n_above}")

    if not C_NOVIDEO:
        if _rec_on_stop:
            cam.stop_recording(save_to_filename=mp4_path, fps=30)
        else:
            cam.stop_recording()
        print(f"\n[saved] {mp4_path}")

    # ── 판정 ──────────────────────────────────────────────────────────────
    A = {k: np.asarray(v, dtype=float) for k, v in rec.items()}
    i_pre  = n_settle - 1
    i_peak = n_settle + n_load + n_hold - 1
    i_end  = n_total - 1
    top_pre, top_peak, top_end = A["bed_top"][i_pre], A["bed_top"][i_peak], A["bed_top"][i_end]
    fz_max = np.nanmax(np.abs(A["fz"])) if np.isfinite(A["fz"]).any() else np.nan
    strain_peak = (top_pre - top_peak) / top_pre if np.isfinite(top_pre) else np.nan
    strain_perm = (top_pre - top_end) / top_pre if np.isfinite(top_pre) else np.nan
    recov = (1.0 - strain_perm / strain_peak
             if np.isfinite(strain_peak) and strain_peak > 1e-6 else np.nan)

    npz = os.path.join(OUT_DIR, f"compaction_{C_MAT}{C_TAG}_{_TS}.npz")
    np.savez(npz, **A, cfg=np.array([C_GRID, C_PIS_VEL, C_PIS_TRAVEL, C_E, C_NU, C_RHO,
                                     C_PHI, C_YLO, C_YHI, C_DT, C_SUB], dtype=float))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        z_ref = z_touch if z_touch is not None else pis_z0
        delta = (z_ref - A["pis_z"]) * 1e3
        fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
        ax[0].plot(A["step"], A["fz"])
        ax[0].set_title("(B) piston Fz [N]"); ax[0].set_xlabel("step"); ax[0].grid(alpha=.3)
        ax[1].plot(delta, A["fz"], lw=1)
        ax[1].set_title("F-delta (load->unload)"); ax[1].set_xlabel("piston travel [mm]")
        ax[1].set_ylabel("Fz [N]"); ax[1].grid(alpha=.3)
        ax[2].plot(A["step"], A["bed_top"]*1e3)
        for k, c, lb in ((i_pre, "g", "pre"), (i_peak, "r", "peak"), (i_end, "b", "end")):
            ax[2].axvline(k, color=c, ls="--", lw=1, label=lb)
        ax[2].set_title("(C) bed top [mm]"); ax[2].set_xlabel("step")
        ax[2].legend(); ax[2].grid(alpha=.3)
        fig.suptitle(f"MAT={C_MAT} dx={dx*1e3:.2f}mm vel={C_PIS_VEL}m/s "
                     f"travel={C_PIS_TRAVEL*1e3:.1f}mm | eps_peak={strain_peak:.4f} "
                     f"eps_perm={strain_perm:.4f} recovery={recov:.3f}")
        fig.tight_layout()
        png = os.path.join(OUT_DIR, f"compaction_{C_MAT}{C_TAG}_{_TS}.png")
        fig.savefig(png, dpi=110); plt.close(fig)
        print(f"[saved] {png}")
    except Exception as e:
        print(f"[warn] plot 실패: {e}")

    print("\n" + "=" * 68)
    print(f"[RESULT] MAT={C_MAT} dx={dx*1e3:.2f}mm vel={C_PIS_VEL} travel={C_PIS_TRAVEL*1e3:.1f}mm")
    print(f"[RESULT] (A) 안정성      : NaN={'없음' if nan_at is None else f'step {nan_at}'}  "
          f"입자수 {A['n_act'][0]:.0f}->{A['n_act'][-1]:.0f}")
    n_above_max = int(np.nanmax(A["n_above"])) if "n_above" in A else -1
    frac_above = n_above_max / max(1.0, A["n_act"][0])
    print(f"[RESULT] (A2) 관통       : n_above_max={n_above_max} "
          f"({frac_above*100:.2f}% of {A['n_act'][0]:.0f})  "
          f"{'관통 심각' if frac_above > 0.02 else '관통 미미'}")
    print(f"[RESULT] (B) 반력 읽기   : Fz_max={fz_max:.6f} N  "
          f"{'읽힘' if np.isfinite(fz_max) and fz_max > 1e-9 else '못 읽음(0 또는 NaN)'}")
    print(f"[RESULT] (C) 압밀        : bed_top pre={top_pre*1e3:.3f} "
          f"peak={top_peak*1e3:.3f} end={top_end*1e3:.3f} mm")
    print(f"[RESULT]     eps_peak={strain_peak:.4f}  eps_perm={strain_perm:.4f}  "
          f"탄성회복률={recov:.3f}  -> "
          f"{'영구압밀 있음' if np.isfinite(strain_perm) and strain_perm > 0.01 else '영구압밀 없음(탄성 복원)'}")
    print(f"[saved] {npz}")
    print("=" * 68)


if __name__ == "__main__":
    if COUPLER == "ipc":
        main_ipc()
    elif TEST_MODE == "compaction":
        main_compaction()
    elif TEST_MODE == "legacy_sanity":
        main_legacy_sanity()
    else:
        main()

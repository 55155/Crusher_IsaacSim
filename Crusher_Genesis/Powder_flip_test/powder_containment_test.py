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
assert TEST_MODE in ("pour", "rigid_probe"), f"TEST_MODE must be pour/rigid_probe, got {TEST_MODE!r}"
PROBE_CUBE_SIZE = 0.004  # 자연 상태 입구 두께(6mm)보다도 작게 — 통과 실패시 순수 기하 문제로 확정.
N_PROBE = 500

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4_PATH = os.path.join(OUT_DIR, f"powder_containment_{BAG_BACKEND}_{_TS}.mp4")

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


if __name__ == "__main__":
    main_ipc() if COUPLER == "ipc" else main()

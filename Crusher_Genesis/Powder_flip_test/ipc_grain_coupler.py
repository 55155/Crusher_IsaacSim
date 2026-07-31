"""
ipc_grain_coupler.py — Genesis의 IPCCoupler를 상속해 pyuipc 네이티브 Particle
구성(점질량)을 알갱이(파우더 낟알)로 등록하는 커플러.

배경(사용자 지시, 2026-07-31): Legacy 커플러의 `mpm_pbd`/`fem_mpm`은 표면
법선/방향 개념이 없는 근접-속도평균 스킴이라(§docs/DigitalTwin.md 조합10),
봉투 입구가 방향 무관 "속도-감쇠 커튼"처럼 작동해 낟알이 안으로 못 들어가고
위에 쌓이는 문제가 있었다. 반면 Genesis의 IPC 래퍼(`ipc_coupler/coupler.py`)
는 FEMSolver/RigidSolver만 알고 다른 재질 커플링은 아예 배선돼 있지 않은데,
pyuipc(`uipc.constitution.Particle`, "point-mass simulation") + pyuipc의
`uipc.geometry.pointcloud()`(dim()==0 SimplicialComplex)는 libuipc 차원에서
FEM.Cloth/Rigid와 **같은 IPC Scene 안에서 동일한 CCD 기반 접촉**으로 다뤄지는
1급 지오메트리다 — 정제가 봉투 안에 성공적으로 들어갔던 것과 같은 메커니즘을
알갱이 각각에 적용할 수 있다는 뜻.

이 파일은 설치된 genesis 패키지(site-packages)를 직접 수정하지 않고,
`IPCCoupler`를 **상속**해 알갱이(Particle) 등록 기능만 추가한다
(`genesis/engine/couplers/ipc_coupler/coupler.py`의 `_add_fem_entities_to_ipc`
/ `_register_contact_pairs` / `_retrieve_fem_states` 패턴을 그대로 따라
`_add_grain_entities_to_ipc` / (오버라이드한) `_register_contact_pairs` /
`_retrieve_grain_states`를 추가).

사용법 — Genesis는 `coupler_options`의 타입(isinstance)만 보고 내부적으로
`IPCCoupler`를 직접 생성하므로(`genesis/engine/simulator.py`), 이 서브클래스를
쓰려면 `gs.Scene()` 생성 직후 `scene.build()` 호출 전에 내부 코클러를
바꿔치기한다(§ 아래 `main_*` 함수들 참고):
    scene = gs.Scene(coupler_options=gs.options.IPCCouplerOptions(...), ...)
    coupler = GrainIPCCoupler(scene._sim, scene._sim.coupler_options)
    scene._sim._coupler = coupler
    ... scene.add_entity(...) ...
    spec_idx = coupler.add_grains(positions, radius=..., mass_density=...)
    scene.build(n_envs=0)
    ...
    scene.step()
    pos = coupler.get_grain_positions(spec_idx)   # (N,3) world 좌표

TEST_MODE 로 두 단계 검증:
    TEST_MODE=sanity python ipc_grain_coupler.py   (기본) — 강체 바닥 + 알갱이만.
        Particle 등록/접촉/상태회수 메커니즘 자체가 crash 없이 동작하는지,
        알갱이가 바닥을 뚫지 않고 멈추는지만 확인하는 최소 재현.
    TEST_MODE=bag python ipc_grain_coupler.py — 실제 봉투(FEM.Cloth) 위에
        알갱이를 부어 담기는지까지 확인(sanity 통과 후 진행).
"""
import os
import sys

import numpy as np

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
import paths  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(_r), "utills"))
from fem_ipc_workarounds import patch_fem_vertex_constraints  # noqa: E402

from datetime import datetime

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")

TEST_MODE = os.environ.get("TEST_MODE", "sanity").lower()
assert TEST_MODE in ("sanity", "bag"), f"TEST_MODE must be sanity/bag, got {TEST_MODE!r}"

GRAIN_RADIUS_M = float(os.environ.get("GRAIN_RADIUS_MM", "1.5")) * 1e-3
GRAIN_RHO = 1500.0
GRAIN_FRICTION = 0.6
N_GRAINS = int(os.environ.get("N_GRAINS", "60"))
DT = 5e-3

# BAG_CONSTRAINT_MODE(사용자 지시, 2026-07-31): "컨택이 안 걸려서가 아니라
# set_vertex_constraints(특히 입구 벌림 target_pos 이동) 자체가 찢어짐/붕괴를
# 유발하는 버그 아니냐"는 가설 검증용 변인통제.
#   full       : 바닥+옆면 고정 + 입구를 target_pos로 벌림(기존 동작, 기본값)
#   fixed_only : 바닥+옆면+입구 전부 "원래 위치"로만 고정, 입구 벌림(이동) 생략
#   none       : 어떤 vertex constraint도 걸지 않음 — 순수 중력 낙하만
BAG_CONSTRAINT_MODE = os.environ.get("BAG_CONSTRAINT_MODE", "full").lower()
assert BAG_CONSTRAINT_MODE in ("full", "fixed_only", "none")

BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag", "Samplebag_seal_pouch3.stl")
BAG_POS = (0.0, 0.0, 0.11)
BAG_EULER = (90, 0, 90)  # full_workflow.py 실측 확정값(§Powder_flip_test/powder_containment_test.py 재사용).


def _npy(x):
    x = x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)
    return x[0] if x.ndim == 3 else x


def _build_grain_coupler_class():
    """GrainIPCCoupler 정의를 지연 임포트 안에 둔다 — `import genesis`(gs.init() 이전에
    uipc 관련 심볼을 끌어오면 초기화 순서 문제가 날 수 있어, 스크립트 실행 시점에
    필요한 시점에만 정의/임포트한다."""
    import uipc
    import uipc.constitution
    import uipc.geometry
    from uipc.geometry import SimplicialComplexSlot

    from genesis.engine.couplers.ipc_coupler.coupler import IPCCoupler
    from genesis.utils.misc import geometric_mean, harmonic_mean
    import genesis as gs

    class GrainIPCCoupler(IPCCoupler):
        """IPCCoupler + pyuipc 네이티브 Particle(점질량) 알갱이 지원.

        `genesis/engine/couplers/ipc_coupler/coupler.py`의 FEM 엔티티 등록
        패턴(`_add_fem_entities_to_ipc`)을 그대로 따라 알갱이를 uipc의
        `pointcloud()` + `Particle` 구성으로 IPC Scene에 추가한다. FEM/Rigid와
        똑같이 ContactTabular에 등록되므로 CCD 기반 접촉(법선/방향 인지)이
        자동으로 적용된다 — Legacy `mpm_pbd`의 방향 무관 근접-스냅과 근본적으로
        다른 지점.
        """

        def __init__(self, simulator, options):
            super().__init__(simulator, options)
            self._grain_specs: list[dict] = []          # add_grains() 호출로 채워짐(build 전용)
            self._ipc_grain_contacts: dict[int, "uipc.core.ContactElement"] = {}
            self._ipc_particle: "uipc.constitution.Particle | None" = None
            self._grain_world_positions: dict[int, list[np.ndarray | None]] = {}

        # ---- 공개 API -------------------------------------------------
        def add_grains(self, positions, radius=GRAIN_RADIUS_M, mass_density=GRAIN_RHO,
                        friction_mu=GRAIN_FRICTION, contact_resistance=None):
            """`scene.build()` 이전에 호출. 알갱이 무리 하나를 등록하고 spec_idx 반환.

            positions : (N,3) array, 초기 world 좌표(겹치지 않게 흩어서 줄 것 —
                IPC 자체는 초기 관통에 대해 안전하지 않을 수 있음).
            radius : Particle 구성의 `thickness` 파라미터 — 접촉용 유효 반경.
            """
            if self._ipc_world is not None:
                gs.raise_exception("add_grains() must be called before scene.build().")
            positions = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
            spec_idx = len(self._grain_specs)
            self._grain_specs.append(dict(
                positions=positions, radius=float(radius), mass_density=float(mass_density),
                friction_mu=float(friction_mu), contact_resistance=contact_resistance,
            ))
            return spec_idx

        def get_grain_positions(self, spec_idx=0, env_idx=0):
            """가장 최근 `couple()`(=scene.step()) 이후 알갱이 world 좌표 (N,3)."""
            return self._grain_world_positions[spec_idx][env_idx]

        # ---- 빌드 단계 오버라이드 --------------------------------------
        def _add_objects_to_ipc(self) -> None:
            # 부모(coupler.py:279)의 순서를 그대로 재현하되, _register_contact_pairs()
            # 전에 알갱이를 추가해야 알갱이용 접촉쌍도 같이 등록된다.
            if self.fem_solver.is_active:
                self._add_fem_entities_to_ipc()
            if self.rigid_solver.is_active:
                self._add_rigid_geoms_to_ipc()
                self._add_articulation_entities_to_ipc()
            self._add_grain_entities_to_ipc()
            self._register_contact_pairs()

        def _add_grain_entities_to_ipc(self) -> None:
            """coupler.py `_add_fem_entities_to_ipc`(293~366줄)와 동일한 패턴 —
            pointcloud + Particle 구성으로 IPC 오브젝트를 만든다."""
            if not self._grain_specs:
                return
            if self._ipc_particle is None:
                self._ipc_particle = uipc.constitution.Particle()
                self._ipc_constitution_tabular.insert(self._ipc_particle)

            for spec_idx, spec in enumerate(self._grain_specs):
                contact = self._ipc_contact_tabular.create(f"grain_contact_{spec_idx}")
                self._ipc_grain_contacts[spec_idx] = contact

                for env_idx in range(self.sim._B):
                    grain_obj = self._ipc_objects.create(f"grain_{spec_idx}_{env_idx}")
                    mesh = uipc.geometry.pointcloud(spec["positions"])
                    # 다른 모든 지오메트리(FEM 314줄, Rigid 462줄)와 동일하게 label_surface
                    # 호출 필요 — 안 하면 is_surf 속성이 없어 접촉 판정 대상에서 완전히
                    # 빠진다(사용자와 함께 확인: 1차 시도가 순수 자유낙하로 crash 없이
                    # 통과했던 원인이 바로 이것이었음, 2026-07-31).
                    uipc.geometry.label_surface(mesh)

                    if self.sim.n_envs > 0:
                        self._ipc_subscenes[env_idx].apply_to(mesh)

                    contact.apply_to(mesh)
                    self._ipc_particle.apply_to(mesh, mass_density=spec["mass_density"], thickness=spec["radius"])

                    meta_attrs = mesh.meta()
                    meta_attrs.create("solver_type", "grain")
                    meta_attrs.create("entity_idx", str(spec_idx))
                    meta_attrs.create("env_idx", str(env_idx))

                    grain_obj.geometries().create(mesh)

        def _register_contact_pairs(self) -> None:
            # 1) 부모(coupler.py:659)가 cloth/fem/abd/ground/no_collision 사이의
            #    기존 쌍을 전부 등록.
            super()._register_contact_pairs()
            if not self._ipc_grain_contacts:
                return

            # 2) 부모와 동일한 방식으로 cloth/fem/abd contact_infos 재구성
            #    (부모가 로컬 변수로만 갖고 있어 재사용 불가 — 그대로 재계산).
            contact_infos = []
            for entity, elem in (*self._ipc_cloth_contacts.items(), *self._ipc_fem_contacts.items()):
                friction = entity.material.friction_mu
                resistance = entity.material.contact_resistance or self.options.contact_resistance
                contact_infos.append((elem, friction, resistance))
            for entity, elem in self._ipc_abd_contacts.items():
                friction = entity.material.coup_friction
                resistance = entity.material.contact_resistance or self.options.contact_resistance
                contact_infos.append((elem, friction, resistance))

            grain_infos = []
            for spec_idx, elem in self._ipc_grain_contacts.items():
                spec = self._grain_specs[spec_idx]
                resistance = spec["contact_resistance"] or self.options.contact_resistance
                grain_infos.append((elem, spec["friction_mu"], resistance))

            # 3) 알갱이 <-> (cloth/fem/abd) — 항상 활성화(알갱이는 전부 만나야 의미가 있음).
            for g_elem, g_friction, g_resistance in grain_infos:
                for elem, friction, resistance in contact_infos:
                    friction_ij = geometric_mean(g_friction, friction)
                    resistance_ij = harmonic_mean(g_resistance, resistance)
                    self._ipc_contact_tabular.insert(g_elem, elem, friction_ij, resistance_ij, True)

            # 4) 알갱이 <-> 알갱이(자기 자신 포함 — 알갱이 더미 자기접촉에 필수).
            for i, (g_elem_i, g_friction_i, g_resistance_i) in enumerate(grain_infos):
                for g_elem_j, g_friction_j, g_resistance_j in grain_infos[i:]:
                    friction_ij = geometric_mean(g_friction_i, g_friction_j)
                    resistance_ij = harmonic_mean(g_resistance_i, g_resistance_j)
                    self._ipc_contact_tabular.insert(g_elem_i, g_elem_j, friction_ij, resistance_ij, True)

            # 5) 알갱이 <-> 바닥(ground plane).
            for entity, ground_elem in self._ipc_ground_contacts.items():
                plane_friction = entity.material.coup_friction
                plane_resistance = entity.material.contact_resistance or self.options.contact_resistance
                for g_elem, g_friction, g_resistance in grain_infos:
                    friction_ground = geometric_mean(g_friction, plane_friction)
                    resistance_ground = harmonic_mean(g_resistance, plane_resistance)
                    self._ipc_contact_tabular.insert(ground_elem, g_elem, friction_ground, resistance_ground, True)

            # 6) 알갱이 <-> no_collision(항상 비활성 — 부모 패턴과 동일하게 마무리).
            for g_elem, *_ in grain_infos:
                self._ipc_contact_tabular.insert(self._ipc_no_collision_contact, g_elem, 0.0, 0.0, False)

        # ---- 스텝 단계 오버라이드 --------------------------------------
        def couple(self, f):
            super().couple(f)  # world.advance()/retrieve() + FEM/Rigid 상태 회수까지 부모가 처리.
            if self.is_active:
                self._retrieve_grain_states()

        def _retrieve_grain_states(self):
            """coupler.py `_retrieve_fem_states`(1015~1050줄)와 동일한 패턴이지만
            dim()==0(점군) 지오메트리를 대상으로 하고, Genesis 쪽에 대응하는
            엔티티가 없으므로 이 커플러 자신의 상태 딕셔너리에 저장한다."""
            if not self._grain_specs:
                return
            from uipc.backend import SceneVisitor

            visitor = SceneVisitor(self._ipc_scene)
            positions_by_spec: dict[int, list[np.ndarray | None]] = {
                i: [None] * self.sim._B for i in range(len(self._grain_specs))
            }
            for geom_slot in visitor.geometries():
                if not isinstance(geom_slot, SimplicialComplexSlot):
                    continue
                geom = geom_slot.geometry()
                if geom.dim() != 0:
                    continue
                meta_attrs = geom.meta()
                st_attr = meta_attrs.find("solver_type")
                if st_attr is None:
                    continue
                (solver_type,) = st_attr.view()
                if solver_type != "grain":
                    continue
                (env_idx,) = map(int, meta_attrs.find("env_idx").view())
                (spec_idx,) = map(int, meta_attrs.find("entity_idx").view())
                (transformed_geom,) = uipc.geometry.apply_transform(geom)
                positions_by_spec[spec_idx][env_idx] = transformed_geom.positions().view().reshape(-1, 3).copy()
            self._grain_world_positions = positions_by_spec

    return GrainIPCCoupler


def main_sanity():
    """TEST_MODE=sanity: 강체 바닥 + 알갱이만 — Particle 등록/접촉/상태회수
    메커니즘 자체가 crash 없이 동작하는지, 알갱이가 바닥을 뚫지 않고 반경만큼
    띄운 채 멈추는지 확인하는 최소 재현."""
    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")
    GrainIPCCoupler = _build_grain_coupler_class()

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=True,
            contact_d_hat=0.0005,
        ),
        show_viewer=False,
    )
    # Genesis는 coupler_options 타입만 보고 내부적으로 기본 IPCCoupler를 생성한다
    # (genesis/engine/simulator.py) — build() 전에 우리 서브클래스로 바꿔치기.
    coupler = GrainIPCCoupler(scene._sim, scene._sim.coupler_options)
    scene._sim._coupler = coupler

    scene.add_entity(
        gs.morphs.Plane(pos=(0, 0, 0)),
        material=gs.materials.Rigid(coup_type="ipc_only"),
    )

    rng = np.random.default_rng(0)
    spacing = GRAIN_RADIUS_M * 4.0  # 초기 관통 방지용 여유 간격.
    grid_n = int(np.ceil(N_GRAINS ** (1 / 3)))
    positions = []
    for i in range(N_GRAINS):
        gx, gy, gz = (i % grid_n), (i // grid_n) % grid_n, (i // (grid_n * grid_n))
        jitter = rng.uniform(-spacing * 0.1, spacing * 0.1, size=3)
        positions.append((
            gx * spacing - grid_n * spacing / 2 + jitter[0],
            gy * spacing - grid_n * spacing / 2 + jitter[1],
            0.05 + gz * spacing + jitter[2],
        ))
    positions = np.array(positions)
    spec_idx = coupler.add_grains(positions, radius=GRAIN_RADIUS_M, mass_density=GRAIN_RHO,
                                   friction_mu=GRAIN_FRICTION)

    # debug=True 필수 — genesis/vis/rasterizer.py:76 skip_markers = not camera.debug 이므로
    # 기본 카메라는 draw_debug_spheres 로 그린 마커를 전부 건너뛴다(알갱이가 Genesis
    # 엔티티가 아니라 마커로만 존재하니 이 옵션 없인 영상에 안 보임 — 2026-07-31 확인).
    cam = scene.add_camera(res=(1024, 768), pos=(0.15, -0.15, 0.15), lookat=(0, 0, 0.02), fov=40, GUI=False,
                           debug=True)

    print(f"\n[build] TEST_MODE=sanity  N_GRAINS={N_GRAINS}  scene.build() 시작...")
    scene.build(n_envs=0)
    print("[build] 성공")

    mp4_path = os.path.join(OUT_DIR, f"ipc_grain_sanity_{_TS}.mp4")
    cam.start_recording(save_to_filename=mp4_path, fps=30)  # Genesis 1.3.1: 파일명/fps는 start_recording으로 이동.
    N_STEPS = 400
    for k in range(N_STEPS):
        scene.step()
        # 알갱이는 uipc pointcloud로만 존재하고 Genesis 엔티티가 아니라서(gs.materials/
        # gs.morphs를 안 거침) 카메라 렌더러가 원래 그릴 대상이 없다 — 매 프레임
        # draw_debug_spheres로 직접 그려줘야 영상에 보인다(사용자 지적, 2026-07-31:
        # "화면에서 파티클이 아예 안 보임").
        scene.clear_debug_objects()
        scene.draw_debug_spheres(coupler.get_grain_positions(spec_idx), radius=GRAIN_RADIUS_M,
                                 color=(0.85, 0.75, 0.55, 1.0))
        cam.render()
        if (k + 1) % 40 == 0:
            pos = coupler.get_grain_positions(spec_idx)
            print(f"[t={(k+1)*DT:6.2f}s] grains min_z={pos[:,2].min():.4f}  max_z={pos[:,2].max():.4f}  "
                  f"mean_z={pos[:,2].mean():.4f}  nan={np.isnan(pos).any()}")
    cam.stop_recording()
    print(f"\n[saved] {mp4_path}")

    pos = coupler.get_grain_positions(spec_idx)
    below_ground = int((pos[:, 2] < -0.001).sum())
    verdict = "OK" if (not np.isnan(pos).any() and below_ground == 0) else "FAIL"
    print("\n" + "=" * 60)
    print(f"[RESULT] TEST_MODE=sanity  N_GRAINS={N_GRAINS}  final min_z={pos[:,2].min():.4f}")
    print(f"[RESULT] below_ground(z<-1mm)={below_ground}/{N_GRAINS}  nan={np.isnan(pos).any()}")
    print(f"[RESULT] verdict={verdict}")
    print("=" * 60)


def main_bag():
    """TEST_MODE=bag: 실제 봉투(FEM.Cloth) 위에 알갱이를 부어 담기는지 확인
    (sanity 통과 후 진행 — powder_containment_test.py main_ipc()의 봉투/입구
    벌림 로직을 재사용)."""
    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")
    patch_fem_vertex_constraints()  # FEMEntity.set_vertex_constraints의 IPC 커플러 체크 로직이
    # 뒤집혀 있는 Genesis 기존 버그 우회(§powder_containment_test.py main_ipc()와 동일 관행).
    GrainIPCCoupler = _build_grain_coupler_class()

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=True,
            contact_d_hat=0.0005,
        ),
        show_viewer=False,
    )
    coupler = GrainIPCCoupler(scene._sim, scene._sim.coupler_options)
    scene._sim._coupler = coupler

    # 사용자 지적(2026-07-31): 봉투를 허공에 띄운 채 바닥+옆면 띠만 고정하면, 그
    # 사이 넓은 몸통 패널은 아무것도 떠받치지 않아 중력만으로 주저앉는다(계측으로
    # 확인 — 자유벨리 최저z가 알갱이 낙하 전부터 이미 바닥 고정band 높이까지
    # 붕괴). 바닥을 봉투 실측 바닥(고정band z=0.0659, run4 계측값) 바로 아래에
    # 둬서 처짐을 바닥이 받아주게 한다 — 실제 사용 환경(회수장치2가 봉투를 받쳐줌)
    # 과 더 비슷한 지지 조건.
    GROUND_Z = BAG_POS[2] - 0.056  # bag 바닥(약 0.0659) 6mm 아래.
    scene.add_entity(
        gs.morphs.Plane(pos=(0, 0, GROUND_Z)),
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

    rng = np.random.default_rng(0)
    mouth_top_z_est = BAG_POS[2] + 0.045
    spacing = GRAIN_RADIUS_M * 4.0
    positions = []
    for i in range(N_GRAINS):
        jitter = rng.uniform(-0.004, 0.004, size=2)
        positions.append((
            BAG_POS[0] + jitter[0], BAG_POS[1] + jitter[1],
            mouth_top_z_est + 0.02 + i * spacing,
        ))
    positions = np.array(positions)
    spec_idx = coupler.add_grains(positions, radius=GRAIN_RADIUS_M, mass_density=GRAIN_RHO,
                                   friction_mu=GRAIN_FRICTION)

    cam = scene.add_camera(res=(1024, 768), pos=(0.30, -0.30, BAG_POS[2] + 0.15),
                           lookat=BAG_POS, fov=40, GUI=False, debug=True)

    print(f"\n[build] TEST_MODE=bag  N_GRAINS={N_GRAINS}  scene.build() 시작...")
    scene.build(n_envs=0)
    print("[build] 성공")

    # 봉투: 바닥+양측면 고정 + 입구 깔때기(powder_containment_test.py main_ipc()와 동일 로직).
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
    if BAG_CONSTRAINT_MODE == "full":
        bag.set_vertex_constraints(verts_idx_local=static_idx, is_soft_constraint=False)
        bag.set_vertex_constraints(verts_idx_local=mouth_idx.tolist(),
                                   target_poss=target_pos[mouth_idx], is_soft_constraint=False)
    elif BAG_CONSTRAINT_MODE == "fixed_only":
        # 입구를 벌리지 않고 전부 "원래 위치"로만 고정 — target_pos 이동(벌림) 자체가
        # 찢어짐/붕괴를 유발하는지 격리하기 위해 이동 없이 원위치 고정만 건다.
        all_fixed_idx = static_idx + mouth_idx.tolist()
        bag.set_vertex_constraints(verts_idx_local=all_fixed_idx, is_soft_constraint=False)
    else:  # "none"
        pass  # 어떤 constraint도 안 검 — 순수 중력 낙하만.
    print(f"[bag] BAG_CONSTRAINT_MODE={BAG_CONSTRAINT_MODE}  바닥+양측면 후보: {len(static_idx)}개, "
          f"입구 후보: {len(mouth_idx)}개")

    # 사용자 가설(2026-07-31) 검증용 계측: "봉투가 바닥까지 늘어나며 찢어지는 것
    # 아니냐"— 하드 제약(static_idx, 원래는 고정된 위치를 유지해야 함)이 실제로
    # 깨지는지, 아니면 제약이 안 걸린 자유 벨리(free_idx, 바닥고정band와
    # 입구band 사이) 부분이 하중으로 늘어나 그 아래로 처지는지 구분해서 로깅.
    n_verts = pos0.shape[0]
    static_idx_arr = np.array(static_idx)
    free_mask = np.ones(n_verts, dtype=bool)
    free_mask[static_idx_arr] = False
    free_mask[mouth_idx] = False
    free_idx_arr = np.where(free_mask)[0]
    static_z0_min, static_z0_max = float(bz[static_idx_arr].min()), float(bz[static_idx_arr].max())
    free_z0_min = float(bz[free_idx_arr].min()) if len(free_idx_arr) else float("nan")
    print(f"[bag] 계측 기준값 — 고정band z=[{static_z0_min:.4f},{static_z0_max:.4f}]  "
          f"자유벨리({len(free_idx_arr)}개) 최저z={free_z0_min:.4f}")

    mp4_path = os.path.join(OUT_DIR, f"ipc_grain_bag_{_TS}.mp4")
    cam.start_recording(save_to_filename=mp4_path, fps=30)  # Genesis 1.3.1: 파일명/fps는 start_recording으로 이동.

    print(f"\n[phase] settle (0.5s) — 봉투만 중력으로 처짐")
    for step in range(100):
        scene.step()
        # 알갱이는 uipc pointcloud로만 존재해 Genesis 카메라가 원래 그릴 대상이
        # 없다 — 매 프레임 draw_debug_spheres로 직접 그려야 영상에 보인다.
        scene.clear_debug_objects()
        scene.draw_debug_spheres(coupler.get_grain_positions(spec_idx), radius=GRAIN_RADIUS_M,
                                 color=(0.85, 0.75, 0.55, 1.0))
        cam.render()
        if (step + 1) % 10 == 0:
            bag_pos_now = _npy(bag.get_state().pos)
            sz_min = float(bag_pos_now[static_idx_arr, 2].min()) if len(static_idx_arr) else float("nan")
            fz_min = float(bag_pos_now[free_idx_arr, 2].min()) if len(free_idx_arr) else float("nan")
            print(f"  [settle t={(step+1)*DT:5.3f}s] 고정band최저z={sz_min:.4f}  자유벨리최저z={fz_min:.4f}")
    bag_bottom_z = float(_npy(bag.get_state().pos)[:, 2].min())
    print(f"[bag] settle 후 바닥 z={bag_bottom_z:.4f}")

    N_DROP = 500
    print(f"\n[phase] drop ({N_DROP*DT:.1f}s) — 알갱이 {N_GRAINS}개 낙하 + 누출 관찰")
    for k in range(N_DROP):
        scene.step()
        scene.clear_debug_objects()
        scene.draw_debug_spheres(coupler.get_grain_positions(spec_idx), radius=GRAIN_RADIUS_M,
                                 color=(0.85, 0.75, 0.55, 1.0))
        cam.render()
        if (k + 1) % 40 == 0:
            pos = coupler.get_grain_positions(spec_idx)
            leaked = pos[:, 2] < (bag_bottom_z - 0.01)
            n_leak = int(leaked.sum())
            bag_pos_now = _npy(bag.get_state().pos)
            static_z_min = float(bag_pos_now[static_idx_arr, 2].min())
            static_z_max = float(bag_pos_now[static_idx_arr, 2].max())
            free_z_min = float(bag_pos_now[free_idx_arr, 2].min()) if len(free_idx_arr) else float("nan")
            print(f"[t={(k+1)*DT:6.2f}s] grains min_z={pos[:,2].min():.4f}  "
                  f"leaked={n_leak}/{N_GRAINS}({100*n_leak/N_GRAINS:.1f}%)  bag_bottom_z={bag_bottom_z:.4f}  "
                  f"고정band z=[{static_z_min:.4f},{static_z_max:.4f}](기준[{static_z0_min:.4f},{static_z0_max:.4f}])  "
                  f"자유벨리최저z={free_z_min:.4f}(기준{free_z0_min:.4f})")

    cam.stop_recording()
    print(f"\n[saved] {mp4_path}")

    pos = coupler.get_grain_positions(spec_idx)
    leaked = pos[:, 2] < (bag_bottom_z - 0.01)
    n_leak = int(leaked.sum())
    frac = n_leak / N_GRAINS
    verdict = "LEAK" if frac > 0.05 else "CONTAINED"
    print("\n" + "=" * 60)
    print(f"[RESULT] TEST_MODE=bag  N_GRAINS={N_GRAINS}  leaked={n_leak}({frac*100:.1f}%)  verdict={verdict}")
    print("=" * 60)


if __name__ == "__main__":
    main_sanity() if TEST_MODE == "sanity" else main_bag()

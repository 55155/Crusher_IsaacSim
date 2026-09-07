"""낟알 등록 순서를 강체보다 **앞으로** 옮기면 ABD 피처가 살아나는가."""
import sys, numpy as np
sys.path.insert(0, r"C:\Crusher_isaacsim\Crusher_Genesis")
sys.path.insert(0, r"C:\Crusher_isaacsim\Crusher_Genesis\Powder_flip_test")
import genesis as gs
from ipc_grain_coupler import _build_grain_coupler_class
gs.init(backend=gs.gpu, logging_level="warning", precision="32", seed=0)
Base = _build_grain_coupler_class()

class GrainFirst(Base):
    def _add_objects_to_ipc(self):
        if self.fem_solver.is_active: self._add_fem_entities_to_ipc()
        self._add_grain_entities_to_ipc()          # <-- 강체보다 먼저
        if self.rigid_solver.is_active:
            self._add_rigid_geoms_to_ipc(); self._add_articulation_entities_to_ipc()
        self._register_contact_pairs()

sc = gs.Scene(sim_options=gs.options.SimOptions(dt=5e-3, gravity=(0,0,-9.81)),
              coupler_options=gs.options.IPCCouplerOptions(two_way_coupling=True,
                  enable_rigid_ground_contact=False, contact_d_hat=1e-4), show_viewer=False)
c = GrainFirst(sc._sim, sc._sim.coupler_options); sc._sim._coupler = c
sc.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(coup_type="ipc_only"))
sc.add_entity(gs.morphs.Box(size=(0.05,0.05,0.02), pos=(0,0,0.10)),
              material=gs.materials.Rigid(coup_type="two_way_soft_constraint"))
rng = np.random.default_rng(0)
c.add_grains(np.c_[rng.uniform(-0.01,0.01,50), rng.uniform(-0.01,0.01,50),
                   rng.uniform(0.20,0.26,50)], radius=1.5e-3, mass_density=1500.0)
try:
    sc.build(n_envs=0); print("[order] 낟알-먼저: build OK")
    for _ in range(30): sc.step()
    print("[order] 낟알-먼저: step OK")
except Exception as e:
    print(f"[order] 낟알-먼저: 실패 {type(e).__name__}: {str(e)[:80]}")

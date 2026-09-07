"""ABD(강체) + uipc Particle(낟알) 이 한 IPC 씬에 공존 가능한가 — 최소 재현."""
import os, sys, numpy as np
sys.path.insert(0, r"C:\Crusher_isaacsim\Crusher_Genesis")
sys.path.insert(0, r"C:\Crusher_isaacsim\Crusher_Genesis\Powder_flip_test")
import genesis as gs
from ipc_grain_coupler import _build_grain_coupler_class
MODE = sys.argv[1]        # abd_only | grain_only | both
gs.init(backend=gs.gpu, logging_level="warning", precision="32", seed=0)
G = _build_grain_coupler_class()
sc = gs.Scene(sim_options=gs.options.SimOptions(dt=5e-3, gravity=(0,0,-9.81)),
              coupler_options=gs.options.IPCCouplerOptions(two_way_coupling=True,
                  enable_rigid_ground_contact=False, contact_d_hat=1e-4),
              show_viewer=False)
c = G(sc._sim, sc._sim.coupler_options); sc._sim._coupler = c
sc.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(coup_type="ipc_only"))
if MODE in ("abd_only", "both"):
    # two_way_soft_constraint 강체 = ABD 바디가 된다(Crusher 벽과 같은 경로).
    sc.add_entity(gs.morphs.Box(size=(0.05,0.05,0.02), pos=(0,0,0.10)),
                  material=gs.materials.Rigid(coup_type="two_way_soft_constraint"))
if MODE in ("ipconly_move", "ipconly_move_grain"):
    # 움직일 수 있는 ipc_only(= fixed 아님). coupler.py 를 보면 이것도
    # _abd_data_by_link 로 구동되므로 uipc 상으로는 ABD 다.
    sc.add_entity(gs.morphs.Box(size=(0.05,0.05,0.02), pos=(0,0,0.10), fixed=False),
                  material=gs.materials.Rigid(coup_type="ipc_only"))
if MODE in ("ipconly_fixed_grain",):
    sc.add_entity(gs.morphs.Box(size=(0.05,0.05,0.02), pos=(0,0,0.10), fixed=True),
                  material=gs.materials.Rigid(coup_type="ipc_only"))
if MODE in ("grain_only", "both", "ipconly_move_grain", "ipconly_fixed_grain"):
    # **격자 배치** — 난수로 뿌리면 서로 겹쳐서 libuipc 초기 거리검사에 걸린다
    # (SimplicialSurfaceDistanceCheck -> "World is not valid, skipping init").
    g = np.array([(i*0.008-0.012, j*0.008-0.012, 0.20+k*0.008)
                  for i in range(4) for j in range(4) for k in range(4)])
    c.add_grains(g, radius=1.5e-3, mass_density=1500.0)
try:
    sc.build(n_envs=0); print(f"[abd] MODE={MODE:10s} build OK")
    for _ in range(30): sc.step()
    print(f"[abd] MODE={MODE:10s} step OK")
except Exception as e:
    print(f"[abd] MODE={MODE:10s} 실패: {type(e).__name__}: {str(e)[:90]}")

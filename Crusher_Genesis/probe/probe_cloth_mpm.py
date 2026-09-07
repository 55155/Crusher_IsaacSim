import numpy as np, genesis as gs, sys, os
mode = sys.argv[1]          # ipc | legacy
gs.init(backend=gs.gpu, logging_level="warning", precision="32", seed=0)
opts = dict(sim_options=gs.options.SimOptions(dt=5e-3, gravity=(0,0,-9.81)),
            mpm_options=gs.options.MPMOptions(lower_bound=(-0.4,-0.4,-0.05), upper_bound=(0.4,0.4,0.8)),
            show_viewer=False)
opts["coupler_options"] = (gs.options.IPCCouplerOptions(two_way_coupling=True) if mode=="ipc"
                           else gs.options.LegacyCouplerOptions(fem_mpm=True, rigid_mpm=True, rigid_fem=True))
sc = gs.Scene(**opts)
sc.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(coup_type="ipc_only") if mode=="ipc" else gs.materials.Rigid())
BAG = r"C:\Crusher_isaacsim\Crusher_Genesis\assets\robots\Samplebag\Samplebag_seal_pouch3.stl"
try:
    bag = sc.add_entity(
        material=gs.materials.FEM.Cloth(E=1e6, nu=0.3, rho=200.0, thickness=1e-4),
        morph=gs.morphs.Mesh(file=BAG, pos=(0,0,0.10), euler=(90,0,90)),
    )
    print("[probe] FEM.Cloth 추가 OK")
except Exception as e:
    print("[probe] FEM.Cloth 추가 실패:", type(e).__name__, str(e)[:160]); bag=None
sand = sc.add_entity(material=gs.materials.MPM.Sand(),
                     morph=gs.morphs.Box(pos=(0.0,0.0,0.40), size=(0.03,0.03,0.03)))
try:
    sc.build(); print("[probe] build OK  coupler =", type(sc.sim.coupler).__name__)
except Exception as e:
    print("[probe] build 실패:", type(e).__name__, str(e)[:200]); raise SystemExit
p = lambda: sand.get_particles_pos().detach().cpu().numpy().reshape(-1,3)[:,2].mean()*1e3
b = (lambda: bag.get_state().pos.detach().cpu().numpy().reshape(-1,3)[:,2].mean()*1e3) if bag else (lambda: float("nan"))
print(f"[probe] t=0      sand_z={p():7.1f}  bag_z={b():7.1f}mm")
for i in range(1,5):
    for _ in range(100): sc.step()
    print(f"[probe] step{i*100:4d}  sand_z={p():7.1f}  bag_z={b():7.1f}mm")

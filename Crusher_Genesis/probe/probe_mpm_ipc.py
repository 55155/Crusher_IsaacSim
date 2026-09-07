import numpy as np, genesis as gs, sys
mode = sys.argv[1]
gs.init(backend=gs.gpu, logging_level="warning", precision="32", seed=0)
opts = dict(sim_options=gs.options.SimOptions(dt=5e-3, gravity=(0,0,-9.81)),
            mpm_options=gs.options.MPMOptions(lower_bound=(-0.3,-0.3,-0.05), upper_bound=(0.3,0.3,0.6)),
            show_viewer=False)
if mode == "ipc":
    opts["coupler_options"] = gs.options.IPCCouplerOptions(two_way_coupling=True)
sc = gs.Scene(**opts)
sc.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(coup_type="ipc_only") if mode=="ipc" else gs.materials.Rigid())
sand = sc.add_entity(
    material=gs.materials.MPM.Sand(),
    morph=gs.morphs.Box(pos=(0.0,0.0,0.30), size=(0.05,0.05,0.05)),
)
print("[probe] MPM 엔티티 추가 OK")
sc.build()
print("[probe] build OK  coupler =", type(sc.sim.coupler).__name__)
print("[probe] mpm_solver.is_active =", sc.sim.mpm_solver.is_active)
z0 = float(sand.get_particles_pos().detach().cpu().numpy().reshape(-1,3)[:,2].mean())
for _ in range(120): sc.step()
z1 = float(sand.get_particles_pos().detach().cpu().numpy().reshape(-1,3)[:,2].mean())
print(f"[probe] sand z {z0*1e3:.1f} -> {z1*1e3:.1f}mm  (바닥에 서면 ~25mm, 뚫으면 음수)")

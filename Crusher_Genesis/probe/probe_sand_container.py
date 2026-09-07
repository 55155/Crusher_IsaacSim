import numpy as np, genesis as gs, sys
kind = sys.argv[1]        # rigidbag | femcube
gs.init(backend=gs.gpu, logging_level="warning", precision="32", seed=0)
sc = gs.Scene(
    sim_options=gs.options.SimOptions(dt=5e-3, gravity=(0,0,-9.81)),
    coupler_options=gs.options.LegacyCouplerOptions(fem_mpm=True, rigid_mpm=True, rigid_fem=True),
    mpm_options=gs.options.MPMOptions(lower_bound=(-0.15,-0.15,-0.02), upper_bound=(0.15,0.15,0.35)),
    show_viewer=False)
sc.add_entity(gs.morphs.Plane())
BAG = r"C:\Crusher_isaacsim\Crusher_Genesis\assets\robots\Samplebag\Samplebag_seal_pouch3.stl"
if kind == "rigidbag":
    holder = sc.add_entity(
        gs.morphs.Mesh(file=BAG, pos=(0,0,0.0), euler=(90,0,90), fixed=True,
                       convexify=False, decimate=False),
        material=gs.materials.Rigid())
    print("[probe] 강체 봉투(convexify=False) 추가")
else:
    holder = sc.add_entity(
        material=gs.materials.FEM.Elastic(E=5e5, nu=0.4, rho=500.0),
        morph=gs.morphs.Box(pos=(0,0,0.05), size=(0.10,0.10,0.02)))
    print("[probe] FEM.Elastic 판 추가")
sand = sc.add_entity(material=gs.materials.MPM.Sand(),
                     morph=gs.morphs.Box(pos=(0.0,0.0,0.20), size=(0.02,0.02,0.02)))
sc.build(); print("[probe] build OK")
P = lambda: sand.get_particles_pos().detach().cpu().numpy().reshape(-1,3)
for i in range(6):
    for _ in range(100): sc.step()
    p = P(); print(f"[probe] step{(i+1)*100:4d}  sand z 평균 {p[:,2].mean()*1e3:7.1f} 최저 {p[:,2].min()*1e3:7.1f}mm  "
                   f"xy반경 최대 {np.linalg.norm(p[:,:2],axis=1).max()*1e3:6.1f}mm")

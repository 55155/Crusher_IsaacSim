import numpy as np, genesis as gs, sys, time
sub = int(sys.argv[1]); gd = float(sys.argv[2]); cpic = sys.argv[3] == "1"
gs.init(backend=gs.gpu, logging_level="warning", precision="32", seed=0)
sc = gs.Scene(
    sim_options=gs.options.SimOptions(dt=5e-3, substeps=sub, gravity=(0,0,-9.81)),
    coupler_options=gs.options.LegacyCouplerOptions(rigid_mpm=True),
    mpm_options=gs.options.MPMOptions(lower_bound=(-0.06,-0.06,-0.01), upper_bound=(0.06,0.06,0.14),
                                      grid_density=gd, enable_CPIC=cpic),
    show_viewer=False)
sc.add_entity(gs.morphs.Plane())
W, T, H, Z0 = 0.030, 0.004, 0.060, 0.005      # 내부 30mm, 벽 4mm, 높이 60mm
for dx,dy,sx,sy in ((W/2+T/2,0,T,W+2*T),(-(W/2+T/2),0,T,W+2*T),(0,W/2+T/2,W+2*T,T),(0,-(W/2+T/2),W+2*T,T)):
    sc.add_entity(gs.morphs.Box(pos=(dx,dy,Z0+H/2), size=(sx,sy,H), fixed=True), material=gs.materials.Rigid())
sc.add_entity(gs.morphs.Box(pos=(0,0,Z0-T/2), size=(W+2*T,W+2*T,T), fixed=True), material=gs.materials.Rigid())
sand = sc.add_entity(material=gs.materials.MPM.Sand(),
                     morph=gs.morphs.Box(pos=(0,0,0.085), size=(0.018,0.018,0.018)))
sc.build()
print(f"[fill] substeps={sub} substep_dt={5e-3/sub:.1e}s gd={gd:.0f} dx={sc.sim.mpm_solver.dx*1e3:.1f}mm "
      f"particle={sc.sim.mpm_solver._particle_size*1e3:.2f}mm n={sand.n_particles} CPIC={cpic}")
P = lambda: sand.get_particles_pos().detach().cpu().numpy().reshape(-1,3)
t0=time.time()
for i in range(5):
    for _ in range(60): sc.step()
    p = P()
    inb = ((np.abs(p[:,0])<W/2)&(np.abs(p[:,1])<W/2)&(p[:,2]<Z0+H)&(p[:,2]>Z0-0.003)).mean()*100
    print(f"[fill] step{(i+1)*60:4d}  용기내부 {inb:5.1f}%  z평균 {p[:,2].mean()*1e3:6.1f} 최저 {p[:,2].min()*1e3:6.1f}  "
          f"xy최대 {np.abs(p[:,:2]).max()*1e3:5.1f}mm  ({time.time()-t0:.0f}s)")

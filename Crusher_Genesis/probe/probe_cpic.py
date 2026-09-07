import numpy as np, genesis as gs, sys
cpic = sys.argv[1] == "1"; gd = float(sys.argv[2]); TW = float(sys.argv[3])*1e-3; MDT = float(sys.argv[4])
gs.init(backend=gs.gpu, logging_level="warning", precision="32", seed=0)
sc = gs.Scene(
    sim_options=gs.options.SimOptions(dt=5e-3, gravity=(0,0,-9.81)),
    coupler_options=gs.options.LegacyCouplerOptions(rigid_mpm=True),
    mpm_options=gs.options.MPMOptions(lower_bound=(-0.06,-0.06,-0.01), upper_bound=(0.06,0.06,0.16),
                                      grid_density=gd, enable_CPIC=cpic, dt=MDT),
    show_viewer=False)
sc.add_entity(gs.morphs.Plane())
# 얇은 벽 용기(벽 2mm) — 봉투 내부 6mm 를 모사. 벽 4장을 Box 로 세운다.
W, T, H, Z0 = 0.030, TW, 0.060, 0.005
for dx, dy, sx, sy in ((W/2+T/2,0,T,W+2*T), (-(W/2+T/2),0,T,W+2*T),
                       (0,W/2+T/2,W+2*T,T), (0,-(W/2+T/2),W+2*T,T)):
    sc.add_entity(gs.morphs.Box(pos=(dx,dy,Z0+H/2), size=(sx,sy,H), fixed=True), material=gs.materials.Rigid())
sc.add_entity(gs.morphs.Box(pos=(0,0,Z0-T/2), size=(W+2*T,W+2*T,T), fixed=True), material=gs.materials.Rigid())
sand = sc.add_entity(material=gs.materials.MPM.Sand(),
                     morph=gs.morphs.Box(pos=(0,0,0.10), size=(0.020,0.020,0.020)))
sc.build()
ps = sc.sim.mpm_solver._particle_size
print(f"[probe] CPIC={cpic} gd={gd:.0f} 벽{TW*1e3:.0f}mm  dx={sc.sim.mpm_solver.dx*1e3:.2f}mm mdt={MDT:.0e}  particle={ps*1e3:.2f}mm  n={sand.n_particles}")
P = lambda: sand.get_particles_pos().detach().cpu().numpy().reshape(-1,3)
for i in range(4):
    for _ in range(100): sc.step()
    p = P(); inside = ((np.abs(p[:,0])<W/2)&(np.abs(p[:,1])<W/2)&(p[:,2]>Z0-0.002)).mean()*100
    print(f"[probe] step{(i+1)*100:4d}  용기 안 {inside:5.1f}%  z평균 {p[:,2].mean()*1e3:6.1f}  "
          f"xy최대 {np.abs(p[:,:2]).max()*1e3:6.1f}mm (벽 안쪽 {W/2*1e3:.0f})")

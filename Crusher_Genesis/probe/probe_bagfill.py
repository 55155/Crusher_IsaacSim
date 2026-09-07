"""봉투 입구 폭 G 를 바꿔가며 MPM.Sand 가 들어가는지 / 입구에서 정체하는지 잰다.
봉투 실치수(폭 64mm, 높이 90mm)를 벽 2mm 상자로 세운다 — 메시 SDF 교란 없이
'입구 폭 대 입자 크기'만 본다."""
import numpy as np, genesis as gs, sys, time
G  = float(sys.argv[1]) * 1e-3      # 입구(두께 방향) 내부 간격
gd = float(sys.argv[2])
gs.init(backend=gs.gpu, logging_level="warning", precision="32", seed=0)
BW, BH, T, Z0 = 0.064, 0.090, 0.002, 0.004      # 봉투 폭/높이/벽두께/바닥z
sc = gs.Scene(
    sim_options=gs.options.SimOptions(dt=5e-3, substeps=50, gravity=(0,0,-9.81)),
    coupler_options=gs.options.LegacyCouplerOptions(rigid_mpm=True),
    mpm_options=gs.options.MPMOptions(lower_bound=(-0.05,-0.06,-0.01), upper_bound=(0.05,0.06,0.19),
                                      grid_density=gd, enable_CPIC=True),
    show_viewer=False)
sc.add_entity(gs.morphs.Plane())
R = lambda **k: sc.add_entity(gs.morphs.Box(fixed=True, **k), material=gs.materials.Rigid())
R(pos=( (G+T)/2,0,Z0+BH/2), size=(T,BW,BH))       # 두께방향 벽 2장 = 입구를 만든다
R(pos=(-(G+T)/2,0,Z0+BH/2), size=(T,BW,BH))
R(pos=(0, (BW+T)/2,Z0+BH/2), size=(G+2*T,T,BH))   # 폭방향 벽 2장
R(pos=(0,-(BW+T)/2,Z0+BH/2), size=(G+2*T,T,BH))
R(pos=(0,0,Z0-T/2), size=(G+2*T,BW+2*T,T))        # 바닥
sand = sc.add_entity(material=gs.materials.MPM.Sand(),
                     morph=gs.morphs.Box(pos=(0,0,0.150), size=(min(G*0.8,0.03),0.030,0.020)))
sc.build()
ps = sc.sim.mpm_solver._particle_size
print(f"[bag] 입구 G={G*1e3:4.1f}mm  gd={gd:.0f}  dx={sc.sim.mpm_solver.dx*1e3:.1f}mm  "
      f"particle={ps*1e3:.2f}mm  G/particle={G/ps:4.1f}  n={sand.n_particles}")
P = lambda: sand.get_particles_pos().detach().cpu().numpy().reshape(-1,3)
top = Z0 + BH          # 입구 평면
t0=time.time()
for i in range(4):
    for _ in range(80): sc.step()
    p = P()
    ins = ((np.abs(p[:,0])<G/2+0.001)&(np.abs(p[:,1])<BW/2)&(p[:,2]<top)&(p[:,2]>Z0-0.003))
    above = (p[:,2] >= top)
    print(f"[bag] step{(i+1)*80:4d}  봉투안 {ins.mean()*100:5.1f}%  입구위 정체 {above.mean()*100:5.1f}%  "
          f"z평균 {p[:,2].mean()*1e3:6.1f} 최저 {p[:,2].min()*1e3:6.1f}mm  ({time.time()-t0:.0f}s)")

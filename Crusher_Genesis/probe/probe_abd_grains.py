"""낟알을 ABD 강체 구(Genesis rigid entity)로 표현 — Crusher(ABD)와 공존하는가.
Particle 을 안 쓰므로 libuipc 의 Particle x ABD 충돌이 원천적으로 없다."""
import os, sys, numpy as np, time
sys.path.insert(0, r"C:\Crusher_isaacsim\Crusher_Genesis")
import genesis as gs
N = int(os.environ.get("N_GRAINS", "40")); R = float(os.environ.get("GRAIN_R_MM", "3.0"))*1e-3
WITH_WALL = os.environ.get("WITH_WALL", "1") == "1"
gs.init(backend=gs.gpu, logging_level="warning", precision="32", seed=0)
sc = gs.Scene(
    sim_options=gs.options.SimOptions(dt=5e-3, gravity=(0,0,-9.81)),
    coupler_options=gs.options.IPCCouplerOptions(
        two_way_coupling=True, contact_friction_enable=True,
        enable_rigid_rigid_contact=os.environ.get("RR","1")=="1",
        enable_rigid_ground_contact=os.environ.get("RG","1")=="1", contact_d_hat=1e-4),
    show_viewer=False)
sc.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(coup_type="ipc_only"))
# 고정 용기(벽 4장) — 낟알이 담길 곳
W,T,H,Z0 = 0.040, 0.004, 0.050, 0.0
for dx,dy,sx,sy in ((W/2+T/2,0,T,W+2*T),(-(W/2+T/2),0,T,W+2*T),(0,W/2+T/2,W+2*T,T),(0,-(W/2+T/2),W+2*T,T)):
    sc.add_entity(gs.morphs.Box(size=(sx,sy,H), pos=(dx,dy,Z0+H/2), fixed=True),
                  material=gs.materials.Rigid(coup_type="ipc_only"))
wall = None
if WITH_WALL:   # 움직이는 ABD — Crusher 벽 역할
    wall = sc.add_entity(gs.morphs.Box(size=(0.004,W,H), pos=(0.060,0,Z0+H/2), fixed=False),
                         material=gs.materials.Rigid(coup_type="two_way_soft_constraint"))
rng = np.random.default_rng(0)
grains = [sc.add_entity(gs.morphs.Sphere(radius=R, pos=(float(x),float(y),float(z))),
                        material=gs.materials.Rigid(coup_type="two_way_soft_constraint", coup_friction=0.6))
          for x,y,z in np.c_[rng.uniform(-0.012,0.012,N), rng.uniform(-0.012,0.012,N),
                             np.linspace(0.06, 0.06+N*2.2*R, N)]]
print(f"[abdg] N={N} 벽={int(WITH_WALL)} RR={os.environ.get('RR','1')} RG={os.environ.get('RG','1')}", end="  ")
t0=time.time()
try:
    sc.build(n_envs=0); print(f"[abdg] build OK ({time.time()-t0:.1f}s)")
except Exception as e:
    print(f"[abdg] build 실패: {type(e).__name__}: {str(e)[:100]}"); raise SystemExit
P = lambda: np.array([g.get_pos().detach().cpu().numpy() for g in grains]).reshape(-1,3)
t0=time.time()
for i in range(4):
    for _ in range(50): sc.step()
    p = P(); inb = ((np.abs(p[:,0])<W/2)&(np.abs(p[:,1])<W/2)).mean()*100
    print(f"[abdg] step{(i+1)*50:4d}  z평균 {p[:,2].mean()*1e3:6.1f} 최저 {p[:,2].min()*1e3:6.1f}mm  "
          f"용기내 {inb:5.1f}%  ({time.time()-t0:.0f}s)")

"""격자 배치 vs 난수 배치 — 초기 배치와 낙하를 영상으로."""
import os, sys, numpy as np
sys.path.insert(0, r"C:\Crusher_isaacsim\Crusher_Genesis")
sys.path.insert(0, r"C:\Crusher_isaacsim\Crusher_Genesis\Powder_flip_test")
import genesis as gs
from ipc_grain_coupler import _build_grain_coupler_class
MODE = sys.argv[1]                      # lattice | random
R, N = 1.5e-3, 64
gs.init(backend=gs.gpu, logging_level="warning", precision="32", seed=0)
G = _build_grain_coupler_class()
sc = gs.Scene(sim_options=gs.options.SimOptions(dt=5e-3, gravity=(0,0,-9.81)),
              coupler_options=gs.options.IPCCouplerOptions(two_way_coupling=True,
                  enable_rigid_ground_contact=False, contact_d_hat=1e-4),
              show_viewer=False)
c = G(sc._sim, sc._sim.coupler_options); sc._sim._coupler = c
sc.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(coup_type="ipc_only"))
if MODE == "lattice":
    p = np.array([(i*0.008-0.012, j*0.008-0.012, 0.05+k*0.008)
                  for i in range(4) for j in range(4) for k in range(4)])
elif MODE == "poisson":
    # **무작위이면서 겹치지 않는 배치** — dart throwing(기각 표집).
    # 최소간격 = 2R + 여유. 격자처럼 규칙적이지 않고, 난수처럼 깨지지도 않는다.
    rng = np.random.default_rng(0); dmin = 2*R + 1.0e-3
    pts = []
    while len(pts) < N:
        q = np.array([rng.uniform(-0.012,0.012), rng.uniform(-0.012,0.012), rng.uniform(0.05,0.13)])
        if all(np.linalg.norm(q-o) >= dmin for o in pts): pts.append(q)
    p = np.array(pts)
else:
    rng = np.random.default_rng(0)
    p = np.c_[rng.uniform(-0.012,0.012,N), rng.uniform(-0.012,0.012,N), rng.uniform(0.05,0.11,N)]
from scipy.spatial.distance import pdist
d = pdist(p); nover = int((d < 2*R).sum())
print(f"[vid] {MODE}: 최근접 {d.min()*1e3:.2f}mm  접촉지름 {2*R*1e3:.1f}mm  겹친쌍 {nover}")
spec = c.add_grains(p, radius=R, mass_density=1500.0, friction_mu=0.6)
cam = sc.add_camera(res=(900, 700), pos=(0.10,-0.12,0.10), lookat=(0,0,0.045),
                    fov=45, GUI=False, debug=True)
try:
    sc.build(n_envs=0); print(f"[vid] {MODE}: build OK")
except Exception as e:
    print(f"[vid] {MODE}: build 실패 — {str(e)[:70]}"); raise SystemExit
out = rf"C:\Crusher_isaacsim\Crusher_Genesis\Powder_flip_test\RESULT\placement_{MODE}.mp4"
cam.start_recording(save_to_filename=out, fps=30)
for k in range(300):
    sc.step()
    sc.clear_debug_objects()
    sc.draw_debug_spheres(c.get_grain_positions(spec), radius=R, color=(0.95,0.6,0.15,1.0))
    cam.render()
cam.stop_recording()
q = c.get_grain_positions(spec)
from scipy.spatial import cKDTree
def nn(a):
    d,_ = cKDTree(a).query(a, k=2); return d[:,1]*1e3
ni, nf = nn(p), nn(q)
print(f"[vid] {MODE}: 종료 z평균 {q[:,2].mean()*1e3:.1f}mm  -> {out}")
print(f"[NN ] {MODE}: 초기 최근접 평균 {ni.mean():5.2f} 표준편차 {ni.std():5.2f}mm  ->  "
      f"종료 평균 {nf.mean():5.2f} 표준편차 {nf.std():5.2f}mm")

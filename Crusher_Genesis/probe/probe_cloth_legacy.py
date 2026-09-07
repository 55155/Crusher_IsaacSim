"""FEM.Cloth + MPM.Sand + LegacyCoupler — substeps 를 바꿔가며 천이 버티는지 본다."""
import numpy as np, genesis as gs, sys, time
sub = int(sys.argv[1]); withsand = sys.argv[2] == "1"
gs.init(backend=gs.gpu, logging_level="warning", precision="32", seed=0)
sc = gs.Scene(
    sim_options=gs.options.SimOptions(dt=5e-3, substeps=sub, gravity=(0,0,-9.81)),
    coupler_options=gs.options.LegacyCouplerOptions(rigid_mpm=True, fem_mpm=True, rigid_fem=True),
    mpm_options=gs.options.MPMOptions(lower_bound=(-0.08,-0.08,-0.02), upper_bound=(0.08,0.08,0.25),
                                      grid_density=200, enable_CPIC=True),
    show_viewer=False)
sc.add_entity(gs.morphs.Plane())
BAG = r"C:\Crusher_isaacsim\Crusher_Genesis\assets\robots\Samplebag\Samplebag_seal_pouch3.stl"
bag = sc.add_entity(material=gs.materials.FEM.Cloth(E=1e6, nu=0.3, rho=200.0, thickness=1e-4),
                    morph=gs.morphs.Mesh(file=BAG, pos=(0,0,0.06), euler=(90,0,90)))
sand = (sc.add_entity(material=gs.materials.MPM.Sand(),
                      morph=gs.morphs.Box(pos=(0,0,0.18), size=(0.02,0.02,0.015))) if withsand else None)
sc.build()
print(f"[cl] substeps={sub} substep_dt={5e-3/sub:.1e}s  sand={'Y' if withsand else 'N'}  "
      f"coupler={type(sc.sim.coupler).__name__}  bag_verts={bag.n_vertices}")
B = lambda: bag.get_state().pos.detach().cpu().numpy().reshape(-1,3)
S = (lambda: sand.get_particles_pos().detach().cpu().numpy().reshape(-1,3)) if sand else None
t0=time.time()
for i in range(4):
    for _ in range(50): sc.step()
    b = B(); nan = not np.isfinite(b).all()
    ext = (b.max(0)-b.min(0))*1e3 if not nan else np.array([np.nan]*3)
    s = f" sand_z={S()[:,2].mean()*1e3:6.1f}" if S else ""
    print(f"[cl] step{(i+1)*50:4d}  bag_NaN={'Y' if nan else 'N'}  "
          f"bag_bbox {ext[0]:5.1f}x{ext[1]:5.1f}x{ext[2]:5.1f}mm (원형 6x64x90){s}  ({time.time()-t0:.0f}s)")
    if nan: break

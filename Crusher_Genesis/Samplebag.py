"""
Samplebag.py — PBD 솔버 단독 검증.

목적:
  M0609/Crusher 없이 PBD cloth (5면체 약봉투) + Rigid Box (내용물) 만 띄워서
  PBD-Rigid coupler 가 안정적으로 동작하는지, 박스가 봉투 안에 정착하는지 확인.

구성:
  · 알루미늄 플레이트 (1m×1m×2cm, 고정, z=0 상단)
  · PBD Cloth 약봉투 (코너 4점 고정 → 자체 형상 유지)
  · Rigid Box (3×0.6×3 cm, ρ=300) 봉투 입구 위에서 낙하

출력:
  Sim_result/Samplebag.mp4
"""
import os
import numpy as np
import trimesh as tm

DT, SUBSTEPS = 1e-3, 10
RENDER_EVERY = 20
N_STEPS = 1500              # 1.5 s — 박스 낙하 + 정착

W, H, D = 0.08, 0.12, 0.01
NW, NH, ND = 6, 9, 2
PARTICLE_SIZE = 2.83e-3

BAG_MOUTH_Z = 0.50
BAG_POS = (0.20, 0.006, BAG_MOUTH_Z - H/2)
BOX_SIZE  = (0.03, 0.006, 0.03)
BOX_RHO   = 300.0
BOX_SPAWN = (0.20, 0.006, 0.575)

_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
STL_PATH = os.path.join(OUT_DIR, "samplebag_envelope.stl")
MP4_PATH = os.path.join(OUT_DIR, "Samplebag.mp4")
PLATE_PATH = os.path.join(_DIR, "robots/assets/aluminum_plate.stl")

CAM_POS    = (0.55, -0.45, 0.65)
CAM_LOOKAT = (0.20, 0.006, 0.48)
CAM_FOV    = 38


def _panel(fn, nu, nv):
    t = []
    for i in range(nu):
        for j in range(nv):
            a, b = fn(i/nu, j/nv), fn((i+1)/nu, j/nv)
            c, d = fn((i+1)/nu, (j+1)/nv), fn(i/nu, (j+1)/nv)
            t += [[a, b, c], [a, c, d]]
    return t


def make_bag():
    tris = []
    tris += _panel(lambda u, v: np.array([u*W, v*H, 0.0]), NW, NH)
    tris += _panel(lambda u, v: np.array([u*W, v*H, D]),   NW, NH)
    tris += _panel(lambda u, v: np.array([u*W, 0.0, v*D]), NW, ND)
    tris += _panel(lambda u, v: np.array([0.0, u*H, v*D]), NH, ND)
    tris += _panel(lambda u, v: np.array([W,   u*H, v*D]), NH, ND)
    v = np.array([p for t in tris for p in t])
    f = np.arange(len(v)).reshape(-1, 3)
    m = tm.Trimesh(vertices=v, faces=f, process=False)
    m.merge_vertices(digits_vertex=7)
    m.vertices -= m.bounding_box.centroid
    m.export(STL_PATH)


def _npy(x): return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)
def _pos_of(b):
    x = _npy(b.get_particles_pos())
    return x[0] if x.ndim == 3 else x


def main(use_viewer: bool = True):
    print("="*60); print(f" Samplebag — PBD cloth + Rigid box (viewer={use_viewer})"); print("="*60)
    make_bag()

    import genesis as gs
    gs.init(backend=gs.cuda, logging_level="warning")

    scene_kwargs = dict(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, -9.81)),
        pbd_options=gs.options.PBDOptions(max_density_solver_iterations=2, particle_size=PARTICLE_SIZE),
        coupler_options=gs.options.LegacyCouplerOptions(rigid_pbd=True),
        vis_options=gs.options.VisOptions(background_color=(0.93, 0.94, 0.96)),
        show_viewer=use_viewer,
    )
    if use_viewer:
        scene_kwargs["viewer_options"] = gs.options.ViewerOptions(
            camera_pos=CAM_POS, camera_lookat=CAM_LOOKAT, camera_fov=CAM_FOV, max_FPS=60)
    scene = gs.Scene(**scene_kwargs)

    scene.add_entity(
        gs.morphs.Mesh(file=PLATE_PATH, fixed=True, pos=(0, 0, 0)),
        material=gs.materials.Rigid(),
        surface=gs.surfaces.Default(color=(0.82, 0.82, 0.85), metallic=0.85, roughness=0.3),
    )
    bag = scene.add_entity(
        material=gs.materials.PBD.Cloth(),
        morph=gs.morphs.Mesh(file=STL_PATH, scale=1.0, pos=BAG_POS, euler=(90, 0, 0)),
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.7, roughness=0.9, double_sided=True),
    )
    box = scene.add_entity(
        material=gs.materials.Rigid(rho=BOX_RHO),
        morph=gs.morphs.Box(size=BOX_SIZE, pos=BOX_SPAWN, fixed=False),
        surface=gs.surfaces.Default(color=(0.85, 0.35, 0.25)),
    )
    cam = scene.add_camera(res=(960, 720), pos=CAM_POS, lookat=CAM_LOOKAT, fov=CAM_FOV, GUI=False)
    scene.build(n_envs=0)

    # 봉투 상단 4 코너 고정 → 형상 유지
    pos0 = _pos_of(bag); z, x, y = pos0[:, 2], pos0[:, 0], pos0[:, 1]
    band = np.where(z >= np.quantile(z, 0.92))[0]
    corners = list({int(i) for i in [
        band[np.argmin(x[band] + y[band])], band[np.argmax(x[band] - y[band])],
        band[np.argmin(-x[band] + y[band])], band[np.argmax(x[band] + y[band])]]})
    print(f"[bag] N={pos0.shape[0]} corners={len(corners)}")
    bag.fix_particles(particles_idx_local=corners)

    cam.start_recording()
    for k in range(N_STEPS):
        scene.step()
        if (k + 1) % RENDER_EVERY == 0:
            cam.render()
        if (k + 1) % 200 == 0:
            bz = _npy(box.get_pos())[2]
            print(f"  step={k+1}/{N_STEPS}  box_z={bz:+.4f}")

    cam.stop_recording(save_to_filename=MP4_PATH, fps=30)
    print(f"[saved] {MP4_PATH}")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=True)

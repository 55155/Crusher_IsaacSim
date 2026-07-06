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

# PBD cloth 강성 (default: stretch=1e-7, bending=1e-5)
# bending compliance 를 키워 굽힘 쉬워짐 → 홀더 클램프 시 자연 접힘
STRETCH_COMPLIANCE = 1e-3   # 1e-7 → 1e-3 (10000× 완화), particle 거리 제약 완화 테스트
BENDING_COMPLIANCE = 1e-3

BAG_MOUTH_Z = 0.50
BAG_POS = (0.20, 0.006, BAG_MOUTH_Z - H/2)
BOX_SIZE  = (0.03, 0.006, 0.03)
BOX_RHO   = 300.0
BOX_SPAWN = (0.20, 0.006, 0.575)

# ── 내용물: 주황 박스 대신 흰 구(primitive Sphere) ─────────────────────────
#   봉투 크기(W,H,D)는 원본 그대로. 구만 교체.
SPHERE_RADIUS = 0.015          # 15 mm (박스 3cm 폭과 유사한 시각 크기)
SPHERE_SPAWN  = BOX_SPAWN      # 동일 낙하 위치

# ── Figure 스틸 (FIG_STILLS=1 일 때만) : 낙하 후 흔들리는 2 프레임 ──────────
FIG_STILLS  = os.environ.get("FIG_STILLS") == "1"
STILL_RES   = (2000, 1500)
STILL_STEPS = (300, 500)       # t≈0.30s, 0.50s — 임팩트 직후 흔들림 구간

_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
FIG_DIR  = os.path.join(OUT_DIR, "samplebag_fig")   # Figure 스틸 출력 (FIG_STILLS)
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
        material=gs.materials.PBD.Cloth(
            stretch_compliance=STRETCH_COMPLIANCE,
            bending_compliance=BENDING_COMPLIANCE,
        ),
        morph=gs.morphs.Mesh(file=STL_PATH, scale=1.0, pos=BAG_POS, euler=(90, 0, 0)),
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.4, roughness=0.9, double_sided=True),
    )
    box = scene.add_entity(
        material=gs.materials.Rigid(rho=BOX_RHO),
        morph=gs.morphs.Sphere(radius=SPHERE_RADIUS, pos=SPHERE_SPAWN, fixed=False),
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.97), roughness=0.4),
    )
    cam = scene.add_camera(res=(960, 720), pos=CAM_POS, lookat=CAM_LOOKAT, fov=CAM_FOV, GUI=False)
    # Figure 용 고해상도 스틸 카메라
    stillcam = None
    if FIG_STILLS:
        os.makedirs(FIG_DIR, exist_ok=True)
        stillcam = scene.add_camera(res=STILL_RES, pos=CAM_POS, lookat=CAM_LOOKAT,
                                    fov=CAM_FOV, GUI=False)
        print(f"[fig] hi-res still cam res={STILL_RES} → {FIG_DIR}")
    scene.build(n_envs=0)

    # 봉투 상단 4 코너 고정 → 형상 유지
    pos0 = _pos_of(bag); z, x, y = pos0[:, 2], pos0[:, 0], pos0[:, 1]
    band = np.where(z >= np.quantile(z, 0.92))[0]
    corners = list({int(i) for i in [
        band[np.argmin(x[band] + y[band])], band[np.argmax(x[band] - y[band])],
        band[np.argmin(-x[band] + y[band])], band[np.argmax(x[band] + y[band])]]})
    print(f"[bag] N={pos0.shape[0]} corners={len(corners)}")
    bag.fix_particles(particles_idx_local=corners)

    # ── PBD stretch 측정 준비 ───────────────────────────────────────────────
    # Genesis PBD 는 STL 을 particle_size 간격으로 재샘플링하므로 particle 인덱스 ≠
    # STL vertex 인덱스. 초기 particle 위치에서 nearest-neighbor (≤ 1.5×particle_size)
    # 쌍을 찾아 "neighbor 거리" 의 변화를 stretch ratio 로 측정.
    from scipy.spatial import cKDTree
    tree = cKDTree(pos0)
    pairs = tree.query_pairs(r=PARTICLE_SIZE * 1.5, output_type="ndarray")
    init_len = np.linalg.norm(pos0[pairs[:, 0]] - pos0[pairs[:, 1]], axis=1)
    print(f"[stretch] {len(pairs)} neighbor pairs (r<{PARTICLE_SIZE*1500:.1f}mm), "
          f"init mean={init_len.mean()*1000:.2f}mm min={init_len.min()*1000:.2f} "
          f"max={init_len.max()*1000:.2f}")

    stretch_log = []
    cam.start_recording()
    for k in range(N_STEPS):
        scene.step()
        if (k + 1) % RENDER_EVERY == 0:
            cam.render()
        if stillcam is not None and (k + 1) in STILL_STEPS:
            from PIL import Image as _I
            rgb = stillcam.render(rgb=True)[0]
            idx = STILL_STEPS.index(k + 1) + 1
            _I.fromarray(np.asarray(rgb)[..., :3].astype("uint8")).save(
                os.path.join(FIG_DIR, f"samplebag_wobble_{idx:02d}.png"))
            print(f"[fig] wobble still {idx} @ step {k+1} (t={(k+1)*DT:.3f}s)")
        if (k + 1) % 200 == 0:
            pos = _pos_of(bag)
            cur_len = np.linalg.norm(pos[pairs[:, 0]] - pos[pairs[:, 1]], axis=1)
            ratio = cur_len / init_len
            bz = _npy(box.get_pos())[2]
            stretch_log.append([(k + 1) * DT, ratio.mean(), ratio.max(), ratio.min(), bz])
            print(f"  t={(k+1)*DT:.3f}s  box_z={bz:+.4f}  "
                  f"stretch mean={ratio.mean():.4f} max={ratio.max():.4f} min={ratio.min():.4f}")

    cam.stop_recording(save_to_filename=MP4_PATH, fps=30)
    print(f"[saved] {MP4_PATH}")

    if stretch_log:
        arr = np.array(stretch_log)
        ss = arr[arr[:, 0] > 0.5]  # 정상상태 t>0.5s
        print("\n[stretch summary]  (t > 0.5s steady-state)")
        print(f"   mean ratio : avg={ss[:, 1].mean():.4f}  peak={ss[:, 1].max():.4f}")
        print(f"   max  ratio : avg={ss[:, 2].mean():.4f}  peak={ss[:, 2].max():.4f}")
        print(f"   min  ratio : avg={ss[:, 3].mean():.4f}  trough={ss[:, 3].min():.4f}")
        print("   해석: ratio>1.0 인장 / <1.0 압축 / 1.0=rest. max ratio<1.05면 매우 단단.")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=True)

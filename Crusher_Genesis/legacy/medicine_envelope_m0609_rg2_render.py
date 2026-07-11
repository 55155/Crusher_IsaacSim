"""
medicine_envelope_m0609_rg2_render.py
M0609 + OnRobot RG2 봉투 파지·리프트를 Genesis 네이티브 카메라로 실사 렌더링(mp4).

물리 시퀀스는 medicine_envelope_m0609_rg2_grasp.py 와 동일.
차이점: matplotlib 대신 Genesis rasterizer 카메라로 프레임 렌더 → 동영상 저장.
  · 헤드리스 가드(Rasterizer.build 패치)를 쓰지 않는다(GL 필요).
  · 디스플레이가 깨어있어야 GL 컨텍스트 생성 가능(잠금/슬립 시 실패).

출력: Sim_result/m0609_rg2_render.mp4  (+ 키프레임 PNG 몇 장)
"""

import os, numpy as np, trimesh as tm
from PIL import Image

W, H, D = 0.08, 0.12, 0.01
NW, NH, ND = 6, 9, 2
PARTICLE_SIZE = 2.83e-3
DT, SUBSTEPS = 5e-4, 10
RENDER_EVERY = 30          # N step 마다 한 프레임 렌더

# 그리퍼를 +z(플랜지 법선)로 정렬하고 아래를 향하게 한 자세 (mount euler 0 -90 0)
Q_ARM_GRASP = np.array([0, -0.40, 1.30, 0, 2.00, 0], float)   # fingerMid≈(0.183,0.006,0.523), S=j2+j3+j5=2.90
# orientation 유지 수직 lift: S(j2+j3+j5)=2.90 고정 → 보간 내내 EE 회전 0 → 봉투 안 기움
Q_ARM_LIFT  = np.array([0, -0.11, 0.60, 0, 2.41, 0], float)   # fingerMid≈(0.183,*,0.649), +12.6cm, dxy 0mm, quat_err≈0
# RG2 핑거 = slide(prismatic, Panda식): 0=닫힘, +=열림
FING_OPEN, FING_CLOSE = 0.04, 0.006
# 봉투를 핑거 끝(fingertip throat) 에 배치 — mouth 가 핑거 길이 구간에 오게
GRASP_XY = np.array([0.20, 0.006]); BAG_MOUTH_Z = 0.50
BAG_POS = (GRASP_XY[0], GRASP_XY[1], BAG_MOUTH_Z - H/2)
# grip 파티클 = 핑거 끝 좁은 영역(적게) → 봉투가 hand 가 아닌 핑거끝에 매달림
THROAT = dict(x=(0.17, 0.27), y=(-0.012, 0.024), z=(0.42, 0.50))
GRIP_LINK = "rg2_left"      # 핑거 링크에 부착(hand 아님 → 너무 높이 매달리는 문제 해결)
# close 후 CSETTLE 로 속도 가라앉힌 뒤 attach → "밀림 없는" tight weld 타이밍
N_SETTLE, N_CLOSE, N_CSETTLE, N_GRASP, N_LIFT, N_HOLD = 400, 600, 400, 200, 2200, 500

# ── 카메라: 처음 넓게 → 파지부터 봉투 트래킹 ──────────────
CAM_WIDE_POS  = np.array([1.15, -1.05, 1.05])   # 넓은 초기 시점
CAM_WIDE_LOOK = np.array([0.25, 0.0, 0.55])
CAM_TRACK_OFF = np.array([0.42, -0.40, 0.20])   # 트래킹 시 봉투COM 기준 카메라 오프셋(근접)

_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
STL_PATH = os.path.join(OUT_DIR, "medicine_envelope_open.stl")
MP4_PATH = os.path.join(OUT_DIR, "m0609_rg2_render.mp4")


def _panel(fn, nu, nv):
    t = []
    for i in range(nu):
        for j in range(nv):
            a, b = fn(i/nu, j/nv), fn((i+1)/nu, j/nv)
            c, d = fn((i+1)/nu, (j+1)/nv), fn(i/nu, (j+1)/nv)
            t += [[a, b, c], [a, c, d]]
    return t

def make_bag():
    tris  = []
    tris += _panel(lambda u, v: np.array([u*W, v*H, 0.0]), NW, NH)
    tris += _panel(lambda u, v: np.array([u*W, v*H, D  ]), NW, NH)
    tris += _panel(lambda u, v: np.array([u*W, 0.0, v*D]), NW, ND)
    tris += _panel(lambda u, v: np.array([0.0, u*H, v*D]), NH, ND)
    tris += _panel(lambda u, v: np.array([W,   u*H, v*D]), NH, ND)
    v = np.array([p for t in tris for p in t]); f = np.arange(len(v)).reshape(-1, 3)
    m = tm.Trimesh(vertices=v, faces=f, process=False)
    m.merge_vertices(digits_vertex=7); m.vertices -= m.bounding_box.centroid
    m.export(STL_PATH)

def npy(x): return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)
def pos_of(bag): x = npy(bag.get_particles_pos()); return x[0] if x.ndim == 3 else x
def lerp(a, b, s): return a + (b - a) * s
def rgb_of(r): a = r[0] if isinstance(r,(tuple,list)) else r; a=npy(a); return a[...,:3].astype("uint8")


def main():
    print("="*60); print(" M0609 + RG2 grasp/lift — Genesis camera render"); print("="*60)
    make_bag()
    import genesis as gs
    gs.init(backend=gs.metal, logging_level="warning")
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, -9.81)),
        pbd_options=gs.options.PBDOptions(
            max_stretch_solver_iterations=25, max_bending_solver_iterations=8,
            max_volume_solver_iterations=3, max_density_solver_iterations=2,
            max_viscosity_solver_iterations=1, particle_size=PARTICLE_SIZE),
        coupler_options=gs.options.LegacyCouplerOptions(rigid_pbd=True),
        vis_options=gs.options.VisOptions(background_color=(0.92, 0.93, 0.95)),
        show_viewer=False)
    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid())
    robot = scene.add_entity(gs.morphs.MJCF(file="robots/m0609_rg2.xml", decimate=False))
    bag = scene.add_entity(material=gs.materials.PBD.Cloth(),
        morph=gs.morphs.Mesh(file=STL_PATH, scale=1.0, pos=BAG_POS, euler=(90, 0, 0)),
        # 흰색 반투명 → 종이 약봉투 느낌
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.7,
                                    roughness=0.9, double_sided=True))
    # 카메라: 그리퍼/봉투를 비스듬히. grasp z≈0.53 ~ lift z≈0.73 커버
    cam = scene.add_camera(res=(960, 720), pos=(0.85, -0.78, 0.85),
                           lookat=(0.20, 0.0, 0.60), fov=46, GUI=False)
    scene.build(n_envs=0)

    names = [l.name for l in robot.links]; li = {n: i for i, n in enumerate(names)}
    hand_idx = robot.get_link(GRIP_LINK).idx
    robot.set_dofs_position(np.concatenate([Q_ARM_GRASP, [FING_OPEN, FING_OPEN]]))

    pos0 = pos_of(bag); z = pos0[:, 2]; x, y = pos0[:, 0], pos0[:, 1]
    band = np.where(z >= np.quantile(z, 0.92))[0]
    corners = list({int(i) for i in [
        band[np.argmin(x[band]+y[band])], band[np.argmax(x[band]-y[band])],
        band[np.argmin(-x[band]+y[band])], band[np.argmax(x[band]+y[band])]]})
    in_throat = ((x>=THROAT['x'][0])&(x<=THROAT['x'][1])&(y>=THROAT['y'][0])&
                 (y<=THROAT['y'][1])&(z>=THROAT['z'][0])&(z<=THROAT['z'][1]))
    grip_idx = np.array([i for i in np.where(in_throat)[0] if i not in corners])
    print(f"[bag] N={pos0.shape[0]} grip={len(grip_idx)} corners={len(corners)}")
    bag.fix_particles(particles_idx_local=corners)

    def bag_com():
        p = pos_of(bag); v = p[~np.isnan(p).any(axis=1)]
        return v.mean(axis=0)

    cam.start_recording()
    keyframes = {}
    step = [0]
    track = [0.0]   # 0=넓게, 1=봉투 트래킹. grasp 부터 1로 램프업
    def update_cam():
        tf = track[0]
        look = lerp(CAM_WIDE_LOOK, bag_com(), tf)
        pos  = lerp(CAM_WIDE_POS, bag_com() + CAM_TRACK_OFF, tf)
        cam.set_pose(pos=pos, lookat=look)

    def run(name, qa0, qa1, f0, f1, n, attach=False, release=False, track_to=None):
        if attach:
            bag.fix_particles_to_link(link_idx=hand_idx, particles_idx_local=grip_idx)
            print(f"[grasp] attach {len(grip_idx)} → {GRIP_LINK}")
        if release:
            bag.release_particle(particles_idx_local=corners)
        for k in range(n):
            s = (k+1)/n
            robot.set_dofs_position(np.concatenate([lerp(qa0, qa1, s), [lerp(f0,f1,s)]*2]))
            scene.step(); step[0] += 1
            if track_to is not None:          # 트래킹 팩터 램프
                track[0] = lerp(track_to[0], track_to[1], s)
            if step[0] % RENDER_EVERY == 0:
                update_cam()
                img = rgb_of(cam.render())
                if name not in keyframes:
                    keyframes[name] = img
        print(f"[phase] {name} done @ step {step[0]}")

    # settle/close: 넓은 시점 (track=0)
    run("settle",  Q_ARM_GRASP, Q_ARM_GRASP, FING_OPEN,  FING_OPEN,  N_SETTLE)
    run("close",   Q_ARM_GRASP, Q_ARM_GRASP, FING_OPEN,  FING_CLOSE, N_CLOSE)
    # close 후 정지 안정화(속도↓) → 밀림 없는 weld 타이밍, 이때 봉투로 줌인
    run("csettle", Q_ARM_GRASP, Q_ARM_GRASP, FING_CLOSE, FING_CLOSE, N_CSETTLE, track_to=(0.0, 1.0))
    # 정지 상태에서 attach(=weld) + 코너 해제
    run("grasp",   Q_ARM_GRASP, Q_ARM_GRASP, FING_CLOSE, FING_CLOSE, N_GRASP, attach=True, release=True)
    # 수직 리프트 (봉투 트래킹 유지)
    run("lift",    Q_ARM_GRASP, Q_ARM_LIFT,  FING_CLOSE, FING_CLOSE, N_LIFT)
    run("hold",    Q_ARM_LIFT,  Q_ARM_LIFT,  FING_CLOSE, FING_CLOSE, N_HOLD)

    cam.stop_recording(save_to_filename=MP4_PATH, fps=20)
    print(f"[saved] {MP4_PATH}")
    for nm, img in keyframes.items():
        p = os.path.join(OUT_DIR, f"m0609_rg2_render_{nm}.png")
        Image.fromarray(img).save(p); print(f"[saved] {p}")
    print("완료.")


if __name__ == "__main__":
    main()

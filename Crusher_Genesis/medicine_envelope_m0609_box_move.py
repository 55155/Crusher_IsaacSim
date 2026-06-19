"""
medicine_envelope_m0609_box_move.py
알루미늄 플레이트 위, 약봉투에 Box 엔티티를 넣고 M0609+RG2 가 집어서 옮기는 시퀀스.

- 바닥: MuJoCo_PlayGround/Ground.stl (1m×1m×2cm 알루미늄 플레이트, 상단 z=0)
- 봉투: 흰색 반투명 PBD cloth (상단 개방)
- 내용물: Genesis 내장 Box 엔티티 (사전정의 primitive → 안정적 충돌)
- 시퀀스: 박스 낙하(in-bag) → 파지 → 수직 lift → 전방 move → 하강 place → release
  (lift/move/place 모두 j2+j3+j5=2.90 고정 → EE orientation 유지)
- 카메라: 넓게 → 봉투 트래킹

출력: Sim_result/m0609_box_move.mp4
"""
import os, numpy as np, trimesh as tm
from PIL import Image

W, H, D = 0.08, 0.12, 0.01
NW, NH, ND = 6, 9, 2
PARTICLE_SIZE = 2.83e-3
DT, SUBSTEPS = 5e-4, 10
RENDER_EVERY = 30

# waypoint (S=j2+j3+j5=2.90 고정 → orientation 유지)
Q_GRASP = np.array([0, -0.40, 1.30, 0, 2.00, 0], float)
Q_LIFT  = np.array([0, -0.11, 0.60, 0, 2.41, 0], float)
Q_MOVE  = np.array([0, -0.05, 0.85, 0, 2.10, 0], float)
Q_PLACE = np.array([0, -0.10, 1.15, 0, 1.85, 0], float)
FING_OPEN, FING_CLOSE = 0.04, 0.006

GRASP_XY = np.array([0.20, 0.006]); BAG_MOUTH_Z = 0.50
BAG_POS = (GRASP_XY[0], GRASP_XY[1], BAG_MOUTH_Z - H/2)
THROAT = dict(x=(0.17, 0.27), y=(-0.012, 0.024), z=(0.42, 0.50))
GRIP_LINK = "rg2_left"

# Box (봉투 1cm 포켓에 들어가도록 얇게)
BOX_SIZE = (0.03, 0.006, 0.03)          # 3 × 0.6 × 3 cm
BOX_RHO  = 300.0
BOX_SPAWN = (0.20, 0.006, 0.575)        # 개방 상단 위에서 낙하

# phase steps
N_DROP, N_CLOSE, N_CSET, N_GRASP, N_LIFT, N_MOVE, N_PLACE, N_REL, N_HOLD = \
    1000, 500, 300, 150, 900, 1200, 700, 300, 300

CAM_WIDE_POS  = np.array([1.15, -1.05, 1.05])
CAM_WIDE_LOOK = np.array([0.27, 0.0, 0.45])
CAM_TRACK_OFF = np.array([0.45, -0.42, 0.18])

_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
STL_PATH = os.path.join(OUT_DIR, "medicine_envelope_open.stl")
MP4_PATH = os.path.join(OUT_DIR, "m0609_box_move.mp4")
PLATE = os.path.join(_DIR, "robots/assets/aluminum_plate.stl")


def _panel(fn, nu, nv):
    t=[]
    for i in range(nu):
        for j in range(nv):
            a,b=fn(i/nu,j/nv),fn((i+1)/nu,j/nv); c,d=fn((i+1)/nu,(j+1)/nv),fn(i/nu,(j+1)/nv)
            t+=[[a,b,c],[a,c,d]]
    return t

def make_bag():
    tris=[]
    tris+=_panel(lambda u,v:np.array([u*W,v*H,0.0]),NW,NH); tris+=_panel(lambda u,v:np.array([u*W,v*H,D]),NW,NH)
    tris+=_panel(lambda u,v:np.array([u*W,0.0,v*D]),NW,ND); tris+=_panel(lambda u,v:np.array([0.0,u*H,v*D]),NH,ND)
    tris+=_panel(lambda u,v:np.array([W,u*H,v*D]),NH,ND)
    v=np.array([p for t in tris for p in t]);f=np.arange(len(v)).reshape(-1,3)
    m=tm.Trimesh(vertices=v,faces=f,process=False);m.merge_vertices(digits_vertex=7);m.vertices-=m.bounding_box.centroid
    m.export(STL_PATH)

def npy(x): return x.cpu().numpy() if hasattr(x,"cpu") else np.asarray(x)
def pos_of(b): x=npy(b.get_particles_pos()); return x[0] if x.ndim==3 else x
def lerp(a,b,s): return a+(b-a)*s
def rgb_of(r): a=r[0] if isinstance(r,(tuple,list)) else r; a=npy(a); return a[...,:3].astype("uint8")


def main():
    print("="*60); print(" M0609+RG2 : box-in-bag pick & move (aluminum plate)"); print("="*60)
    make_bag()
    import genesis as gs
    gs.init(backend=gs.metal, logging_level="warning")
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0,0,-9.81)),
        pbd_options=gs.options.PBDOptions(max_density_solver_iterations=2, particle_size=PARTICLE_SIZE),
        coupler_options=gs.options.LegacyCouplerOptions(rigid_pbd=True),
        vis_options=gs.options.VisOptions(background_color=(0.93,0.94,0.96)),
        show_viewer=False)
    # 알루미늄 플레이트 (사전정의 mesh, 고정 rigid). 슬랩이라 convexify 시 안정.
    scene.add_entity(gs.morphs.Mesh(file=PLATE, fixed=True, pos=(0,0,0)),
                     material=gs.materials.Rigid(),
                     surface=gs.surfaces.Default(color=(0.82,0.82,0.85), metallic=0.85, roughness=0.3))
    robot = scene.add_entity(gs.morphs.MJCF(file="robots/m0609_rg2.xml", decimate=False))
    bag = scene.add_entity(material=gs.materials.PBD.Cloth(),
        morph=gs.morphs.Mesh(file=STL_PATH, scale=1.0, pos=BAG_POS, euler=(90,0,0)),
        surface=gs.surfaces.Default(color=(0.97,0.97,0.95), opacity=0.7, roughness=0.9, double_sided=True))
    box = scene.add_entity(material=gs.materials.Rigid(rho=BOX_RHO),
        morph=gs.morphs.Box(size=BOX_SIZE, pos=BOX_SPAWN, fixed=False),
        surface=gs.surfaces.Default(color=(0.85,0.35,0.25)))
    cam = scene.add_camera(res=(960,720), pos=CAM_WIDE_POS, lookat=CAM_WIDE_LOOK, fov=46, GUI=False)
    scene.build(n_envs=0)

    names=[l.name for l in robot.links]; li={n:i for i,n in enumerate(names)}
    grip_link_idx = robot.get_link(GRIP_LINK).idx
    robot.set_dofs_position(np.concatenate([Q_GRASP,[FING_OPEN,FING_OPEN]]))

    pos0=pos_of(bag); z,x,y=pos0[:,2],pos0[:,0],pos0[:,1]
    band=np.where(z>=np.quantile(z,0.92))[0]
    corners=list({int(i) for i in [band[np.argmin(x[band]+y[band])],band[np.argmax(x[band]-y[band])],
                  band[np.argmin(-x[band]+y[band])],band[np.argmax(x[band]+y[band])]]})
    thr=((x>=THROAT['x'][0])&(x<=THROAT['x'][1])&(y>=THROAT['y'][0])&(y<=THROAT['y'][1])&
         (z>=THROAT['z'][0])&(z<=THROAT['z'][1]))
    grip_idx=np.array([i for i in np.where(thr)[0] if i not in corners])
    print(f"[bag] N={pos0.shape[0]} grip={len(grip_idx)} corners={len(corners)}")
    bag.fix_particles(particles_idx_local=corners)

    def bag_com():
        p=pos_of(bag); v=p[~np.isnan(p).any(axis=1)]; return v.mean(axis=0)

    cam.start_recording(); keyframes={}; step=[0]; track=[0.0]
    def update_cam():
        tf=track[0]
        cam.set_pose(pos=lerp(CAM_WIDE_POS, bag_com()+CAM_TRACK_OFF, tf),
                     lookat=lerp(CAM_WIDE_LOOK, bag_com(), tf))
    def run(name, qa0, qa1, f0, f1, n, attach=False, release=False, drop_release=False, track_to=None):
        if attach:
            bag.fix_particles_to_link(link_idx=grip_link_idx, particles_idx_local=grip_idx)
            print(f"[grasp] attach {len(grip_idx)} → {GRIP_LINK}")
        if release:
            bag.release_particle(particles_idx_local=corners)
        if drop_release:
            bag.release_particle(particles_idx_local=grip_idx)
            print("[place] released bag from gripper")
        for k in range(n):
            s=(k+1)/n
            robot.set_dofs_position(np.concatenate([lerp(qa0,qa1,s),[lerp(f0,f1,s)]*2]))
            scene.step(); step[0]+=1
            if track_to is not None: track[0]=lerp(track_to[0],track_to[1],s)
            if step[0]%RENDER_EVERY==0:
                update_cam(); img=rgb_of(cam.render())
                if name not in keyframes: keyframes[name]=img
        print(f"[phase] {name} @ {step[0]}")

    run("dropin",  Q_GRASP, Q_GRASP, FING_OPEN, FING_OPEN, N_DROP, track_to=(0.0,0.6))
    run("close",   Q_GRASP, Q_GRASP, FING_OPEN, FING_CLOSE, N_CLOSE, track_to=(0.6,1.0))
    run("csettle", Q_GRASP, Q_GRASP, FING_CLOSE, FING_CLOSE, N_CSET)
    run("grasp",   Q_GRASP, Q_GRASP, FING_CLOSE, FING_CLOSE, N_GRASP, attach=True, release=True)
    run("lift",    Q_GRASP, Q_LIFT,  FING_CLOSE, FING_CLOSE, N_LIFT)
    run("move",    Q_LIFT,  Q_MOVE,  FING_CLOSE, FING_CLOSE, N_MOVE)
    run("place",   Q_MOVE,  Q_PLACE, FING_CLOSE, FING_CLOSE, N_PLACE)
    run("release", Q_PLACE, Q_PLACE, FING_CLOSE, FING_OPEN, N_REL, drop_release=True)
    run("hold",    Q_PLACE, Q_PLACE, FING_OPEN, FING_OPEN, N_HOLD)

    cam.stop_recording(save_to_filename=MP4_PATH, fps=20)
    print(f"[saved] {MP4_PATH}")
    for nm,img in keyframes.items():
        Image.fromarray(img).save(os.path.join(OUT_DIR,f"m0609_box_move_{nm}.png"))
    print("완료.")


if __name__ == "__main__":
    main()

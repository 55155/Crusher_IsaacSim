"""
Crushing.py — 시스템 전체 통합 시뮬레이션.

구성:
  · 알루미늄 플레이트 (1m×1m×2cm, 상단 z=0, 고정)
  · M0609 6-DOF 매니퓰레이터 + OnRobot RG2 그리퍼  (robots/m0609_rg2.xml)
  · PBD 약봉투 (5면체 cloth, 상단 개방)  + 내부 Rigid 박스
  · Crusher_IsaacSim (Wall_1 슬라이드 + 크랭크슬라이더 메커니즘) ─ 로봇 정면, Wall_1 -x 향함

시퀀스:
  dropin → close → csettle → grasp → lift → move → place → release → hold
  (j2+j3+j5=2.90 고정 → EE orientation 보존)

핵심 처리:
  · Crusher MJCF 런타임 패치 (lock_crank equality 제거, weld solref/solimp 강화, ground 제거)
  · Crusher 엔티티에 decimate=False, convexify=False, surface(smooth=False) → 원본 STL 보존

출력:
  Sim_result/Crushing.mp4 + 페이즈별 키프레임 PNG
"""
import os, shutil, tempfile
import xml.etree.ElementTree as ET
import numpy as np
import trimesh as tm
from PIL import Image

# ── 시뮬 옵션 ────────────────────────────────────────────────────────────────
DT, SUBSTEPS = 1e-3, 10
RENDER_EVERY = 15

# ── 봉투 (cloth 패널) ────────────────────────────────────────────────────────
W, H, D = 0.08, 0.12, 0.01
NW, NH, ND = 6, 9, 2
PARTICLE_SIZE = 2.83e-3

# ── 로봇 waypoint (S=j2+j3+j5=2.90 → EE 자세 유지) ────────────────────────
Q_GRASP = np.array([0, -0.40, 1.30, 0, 2.00, 0], float)
Q_LIFT  = np.array([0, -0.11, 0.60, 0, 2.41, 0], float)
Q_MOVE  = np.array([0, -0.05, 0.85, 0, 2.10, 0], float)
Q_PLACE = np.array([0, -0.10, 1.15, 0, 1.85, 0], float)
FING_OPEN, FING_CLOSE = 0.04, 0.006

# ── 봉투/박스 spawn ─────────────────────────────────────────────────────────
GRASP_XY    = np.array([0.20, 0.006]); BAG_MOUTH_Z = 0.50
BAG_POS     = (GRASP_XY[0], GRASP_XY[1], BAG_MOUTH_Z - H/2)
# 파지 대상: 봉투 "가로 중앙 + 상단" 좁은 strip (3D throat 박스 대신)
#   x: bag 중앙선 ±5mm   y: bag 두께 전체 (핑거 사이)   z: top ±5mm
GRIP_X_WIDTH    = 0.005     # 가로 중앙 strip 폭
GRIP_Z_FROM_TOP = 0.005     # 상단에서 두께
GRIP_LINK       = "rg2_left"
BOX_SIZE    = (0.03, 0.006, 0.03)
BOX_RHO     = 300.0
BOX_SPAWN   = (0.20, 0.006, 0.575)

# ── 페이즈 step (DT 변경에 맞춰 절반으로) ───────────────────────────────────
N_DROP, N_CLOSE, N_CSET, N_GRASP, N_LIFT, N_MOVE, N_PLACE, N_REL, N_HOLD = \
    500, 250, 150, 75, 450, 600, 350, 150, 150

# ── 카메라 ──────────────────────────────────────────────────────────────────
CAM_WIDE_POS  = np.array([1.15, -1.05, 1.05])
CAM_WIDE_LOOK = np.array([0.27, 0.0, 0.45])
CAM_TRACK_OFF = np.array([0.45, -0.42, 0.18])

# ── Crusher 배치 (매니퓰레이터 정면 + 작업영역 높이로 elevate) ────────────
#   원본 Wall_1 local z≈17mm → 받침대 효과로 z=0.4 띄움 → Wall_1 world z≈0.42.
#   M0609 reach 영역 (z 0.2~1.2 m) 내로 진입.
CRUSHER_POS   = (0.55, 0.0, 0.4)
CRUSHER_EULER = (0.0, 0.0, 90.0)

# ── PBD 봉투 강성 (Samplebag 에서 검증: 박스 안전 + 유연성 ↑) ──────────────
STRETCH_COMPLIANCE = 1e-3
BENDING_COMPLIANCE = 1e-3

# ── Wall_1 정면 placement: Wall_1 world pos + offset → IK Q_PLACE ──────────
WALL_LINK_NAME    = "L2_Left_Wall1_1"
EE_LINK_NAME      = "rg2_hand"
# Wall_1 face 기준 봉투 hang 위치 (front=음의 X, slightly above wall)
EE_OFFSET_M       = np.array([-0.05, 0.0, 0.05])

# ── 경로 ────────────────────────────────────────────────────────────────────
_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
STL_PATH = os.path.join(OUT_DIR, "medicine_envelope_open.stl")
MP4_PATH = os.path.join(OUT_DIR, "Crushing.mp4")
PLATE_PATH = os.path.join(_DIR, "robots/assets/aluminum_plate.stl")
ROBOT_MJCF = "robots/m0609_rg2.xml"
CRUSHER_SRC_XML = os.path.join(_DIR, "MJCF", "Crusher_IsaacSim_colored.xml")


# ─────────────────────────────── helpers ────────────────────────────────────
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


WALL_GEOMS_TO_ENABLE = {"L1_Wall1_1", "L1_Wall2_1", "L2_Wall3_1"}
L7_LINK3_COM = "0.006 0 -0.005"


def patch_crusher_mjcf(src, dst,
                        eq_solref="0.0002 50",
                        eq_solimp="0.999 0.99999 1e-5"):
    """원본 Crusher MJCF → Genesis 호환 사본.

    - <equality><joint> (lock_crank polycoef): 제거
    - <equality><weld> 유지, solref/solimp 매우 빡빡한 값으로 교체
        timeconst=0.2ms, dampratio=50, solimp 최대 impedance
    - <geom name="ground">: 알루미늄 플레이트와 중복이라 제거
    - 벽 geom (L1_Wall1_1, L1_Wall2_1, L2_Wall3_1) 의 충돌 활성화
    - L7_Link3_1 의 dubious CoM 을 bbox 중심으로 교체 (URDF 변환 오류 보정)
    """
    tree = ET.parse(src); root = tree.getroot()
    eq = root.find("equality")
    if eq is not None:
        for j in list(eq.findall("joint")):
            eq.remove(j)
        for w in eq.findall("weld"):
            w.set("solref", eq_solref)
            w.set("solimp", eq_solimp)
    wb = root.find("worldbody")
    if wb is not None:
        for g in list(wb.findall("geom")):
            if g.get("name") == "ground":
                wb.remove(g)
        for g in wb.iter("geom"):
            if g.get("mesh") in WALL_GEOMS_TO_ENABLE:
                g.attrib.pop("contype", None)
                g.attrib.pop("conaffinity", None)
        for body in wb.iter("body"):
            if body.get("name") == "L7_Link3_1":
                inertial = body.find("inertial")
                if inertial is not None:
                    inertial.set("pos", L7_LINK3_COM)
    tree.write(dst)


def _prepare_crusher_mjcf():
    tmp_dir = tempfile.mkdtemp(prefix="crusher_mjcf_")
    src_dir = os.path.dirname(CRUSHER_SRC_XML)
    for f in os.listdir(src_dir):
        s = os.path.join(src_dir, f)
        if os.path.isfile(s):
            shutil.copy2(s, os.path.join(tmp_dir, f))
    dst = os.path.join(tmp_dir, "Crusher_genesis.xml")
    patch_crusher_mjcf(CRUSHER_SRC_XML, dst)
    return dst


def _npy(x): return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)
def _pos_of(b):
    x = _npy(b.get_particles_pos())
    return x[0] if x.ndim == 3 else x
def _lerp(a, b, s): return a + (b - a) * s
def _rgb_of(r):
    a = r[0] if isinstance(r, (tuple, list)) else r
    a = _npy(a)
    return a[..., :3].astype("uint8")


def main(use_viewer: bool = True):
    print("="*60); print(f" Crushing — full pick & move + Crusher (viewer={use_viewer})"); print("="*60)
    make_bag()
    crusher_xml = _prepare_crusher_mjcf()
    print(f"[crusher] patched MJCF → {crusher_xml}")

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
            camera_pos=tuple(CAM_WIDE_POS), camera_lookat=tuple(CAM_WIDE_LOOK),
            camera_fov=46, max_FPS=60)
    scene = gs.Scene(**scene_kwargs)

    # 알루미늄 플레이트
    scene.add_entity(
        gs.morphs.Mesh(file=PLATE_PATH, fixed=True, pos=(0, 0, 0)),
        material=gs.materials.Rigid(),
        surface=gs.surfaces.Default(color=(0.82, 0.82, 0.85), metallic=0.85, roughness=0.3),
    )
    # M0609 + RG2
    robot = scene.add_entity(gs.morphs.MJCF(file=ROBOT_MJCF, decimate=False))
    # Crusher (정면, Wall_1 -x 향함, faceted 정밀 렌더)
    crusher = scene.add_entity(
        gs.morphs.MJCF(file=crusher_xml, pos=CRUSHER_POS, euler=CRUSHER_EULER,
                       decimate=False, convexify=False),
        surface=gs.surfaces.Default(smooth=False),
    )
    # PBD 약봉투 — Samplebag 검증된 유연 강성
    bag = scene.add_entity(
        material=gs.materials.PBD.Cloth(
            stretch_compliance=STRETCH_COMPLIANCE,
            bending_compliance=BENDING_COMPLIANCE,
        ),
        morph=gs.morphs.Mesh(file=STL_PATH, scale=1.0, pos=BAG_POS, euler=(90, 0, 0)),
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.7, roughness=0.9, double_sided=True),
    )
    # 내용물 박스
    scene.add_entity(
        material=gs.materials.Rigid(rho=BOX_RHO),
        morph=gs.morphs.Box(size=BOX_SIZE, pos=BOX_SPAWN, fixed=False),
        surface=gs.surfaces.Default(color=(0.85, 0.35, 0.25)),
    )
    cam = scene.add_camera(res=(960, 720), pos=tuple(CAM_WIDE_POS),
                           lookat=tuple(CAM_WIDE_LOOK), fov=46, GUI=False)
    scene.build(n_envs=0)

    grip_link_idx = robot.get_link(GRIP_LINK).idx
    robot.set_dofs_position(np.concatenate([Q_GRASP, [FING_OPEN, FING_OPEN]]))

    # ── Wall_1 world pos 분석 (Crusher 좌표계 검증) ─────────────────────────
    wall_link  = crusher.get_link(WALL_LINK_NAME)
    ee_link    = robot.get_link(EE_LINK_NAME)
    wall_pos   = _npy(wall_link.get_pos())

    # MJCF body 'L2_Left_Wall1_1' 의 원본 pos (Crusher local frame, before transform)
    wall_body_mjcf = np.array([-0.017802, 0.286278, 0.016542])
    yaw_deg = CRUSHER_EULER[2]; yaw = np.radians(yaw_deg)
    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw),  np.cos(yaw), 0],
                      [0, 0, 1]])
    wall_expected = R_yaw @ wall_body_mjcf + np.array(CRUSHER_POS)
    print(f"\n[wall-analysis]")
    print(f"  MJCF body L2_Left_Wall1_1 pos      = {wall_body_mjcf}  (Crusher local)")
    print(f"  CRUSHER_POS={CRUSHER_POS}  EULER yaw={yaw_deg}°")
    print(f"  R_yaw @ body + CRUSHER_POS         = {wall_expected}  (manual transform)")
    print(f"  Genesis link.get_pos()             = {wall_pos}  (Genesis built-in)")
    print(f"  diff (manual - genesis)            = {wall_expected - wall_pos}\n")

    ee_target  = wall_pos + EE_OFFSET_M
    print(f"[place] Wall_1 world={wall_pos}  ee_target={ee_target}")

    # Genesis IK 가 이 robot/target 조합에서 비정상 — manual brute search 로 대체.
    # j2+j3+j5=2.90 invariant 유지 (EE 수직 자세 보존), j4=j6=0 고정.
    # j1, j2, j3 grid scan → EE world pos 측정 → target 최근접 Q 채택.
    def _find_q_place(target):
        best_q, best_err, best_ee = None, float("inf"), None
        for j1 in np.linspace(-0.8, 0.8, 17):
            for j2 in np.linspace(-1.0, 0.5, 16):
                for j3 in np.linspace(0.3, 2.0, 18):
                    j5 = 2.90 - j2 - j3
                    if not (-1.0 < j5 < 3.5):
                        continue
                    q = np.array([j1, j2, j3, 0.0, j5, 0.0])
                    robot.set_dofs_position(np.concatenate([q, [FING_CLOSE, FING_CLOSE]]))
                    ee = _npy(ee_link.get_pos())
                    e = float(np.linalg.norm(ee - target))
                    if e < best_err:
                        best_err, best_q, best_ee = e, q.copy(), ee
        return best_q, best_err, best_ee

    Q_PLACE_IK, err_m, ee_at_ik = _find_q_place(ee_target)
    err_mm = err_m * 1000
    print(f"[place] best Q (grid scan) = {Q_PLACE_IK}")
    print(f"[place] EE @ best Q        = {ee_at_ik}  (target {ee_target}, err {err_mm:.1f} mm)")

    # 다시 원래 자세 (sim 시작점) 으로 복귀
    robot.set_dofs_position(np.concatenate([Q_GRASP, [FING_OPEN, FING_OPEN]]))

    pos0 = _pos_of(bag); z, x, y = pos0[:, 2], pos0[:, 0], pos0[:, 1]
    band = np.where(z >= np.quantile(z, 0.92))[0]
    corners = list({int(i) for i in [
        band[np.argmin(x[band] + y[band])], band[np.argmax(x[band] - y[band])],
        band[np.argmin(-x[band] + y[band])], band[np.argmax(x[band] + y[band])]]})

    # 가로 중앙 strip + 상단 → 가장 좁은 그립 영역
    x_center = float(GRASP_XY[0])
    z_top    = float(z.max())
    mid_strip = (np.abs(x - x_center) < GRIP_X_WIDTH) & (z > z_top - GRIP_Z_FROM_TOP)
    grip_idx = np.array([i for i in np.where(mid_strip)[0] if i not in corners])
    print(f"[bag] N={pos0.shape[0]}  grip(mid-top strip)={len(grip_idx)}  corners={len(corners)}")
    print(f"      grip strip: x∈[{x_center-GRIP_X_WIDTH:.3f},{x_center+GRIP_X_WIDTH:.3f}], "
          f"z>{z_top-GRIP_Z_FROM_TOP:.3f}")
    bag.fix_particles(particles_idx_local=corners)

    def bag_com():
        p = _pos_of(bag); v = p[~np.isnan(p).any(axis=1)]
        return v.mean(axis=0)

    cam.start_recording(); keyframes = {}; step = [0]; track = [0.0]

    def update_cam():
        tf = track[0]
        cam.set_pose(pos=_lerp(CAM_WIDE_POS, bag_com() + CAM_TRACK_OFF, tf),
                     lookat=_lerp(CAM_WIDE_LOOK, bag_com(), tf))

    def run(name, qa0, qa1, f0, f1, n, attach=False, release=False,
            drop_release=False, track_to=None):
        if attach:
            bag.fix_particles_to_link(link_idx=grip_link_idx, particles_idx_local=grip_idx)
            print(f"[grasp] attach {len(grip_idx)} → {GRIP_LINK}")
        if release:
            bag.release_particle(particles_idx_local=corners)
        if drop_release:
            bag.release_particle(particles_idx_local=grip_idx)
            print("[place] released bag from gripper")
        for k in range(n):
            s = (k + 1) / n
            robot.set_dofs_position(np.concatenate([_lerp(qa0, qa1, s), [_lerp(f0, f1, s)] * 2]))
            scene.step(); step[0] += 1
            if track_to is not None:
                track[0] = _lerp(track_to[0], track_to[1], s)
            if step[0] % RENDER_EVERY == 0:
                update_cam(); img = _rgb_of(cam.render())
                if name not in keyframes:
                    keyframes[name] = img
        print(f"[phase] {name} @ {step[0]}")

    run("dropin",  Q_GRASP, Q_GRASP, FING_OPEN, FING_OPEN, N_DROP, track_to=(0.0, 0.6))
    run("close",   Q_GRASP, Q_GRASP, FING_OPEN, FING_CLOSE, N_CLOSE, track_to=(0.6, 1.0))
    run("csettle", Q_GRASP, Q_GRASP, FING_CLOSE, FING_CLOSE, N_CSET)
    run("grasp",   Q_GRASP, Q_GRASP, FING_CLOSE, FING_CLOSE, N_GRASP, attach=True, release=True)
    run("lift",    Q_GRASP, Q_LIFT,  FING_CLOSE, FING_CLOSE, N_LIFT)
    run("move",    Q_LIFT,  Q_MOVE,  FING_CLOSE, FING_CLOSE, N_MOVE)
    run("place",   Q_MOVE,  Q_PLACE_IK, FING_CLOSE, FING_CLOSE, N_PLACE)

    # ── Phase A verify: 봉투가 release 직전에 Wall_1 입구에 있는지 ──────────
    bag_at_place = bag_com()
    ee_at_place  = _npy(ee_link.get_pos())
    wall_at_place = _npy(wall_link.get_pos())
    err_ee_target = ee_at_place - ee_target
    err_bag_wall  = bag_at_place - wall_at_place
    print(f"\n[verify @ end-of-place, before release]")
    print(f"  EE  world  = {ee_at_place}  (target {ee_target}, err {np.linalg.norm(err_ee_target)*1000:.1f} mm)")
    print(f"  bag com    = {bag_at_place}")
    print(f"  wall_1 com = {wall_at_place}")
    print(f"  bag - wall = {err_bag_wall}  (xy_dist {np.linalg.norm(err_bag_wall[:2])*1000:.1f}mm, "
          f"dz {err_bag_wall[2]*1000:+.1f}mm)")

    run("release", Q_PLACE_IK, Q_PLACE_IK, FING_CLOSE, FING_OPEN, N_REL, drop_release=True)
    run("hold",    Q_PLACE_IK, Q_PLACE_IK, FING_OPEN, FING_OPEN, N_HOLD)

    print(f"\n[verify @ end-of-hold]  bag final com = {bag_com()}  (will fall w/o holder clamp)")

    cam.stop_recording(save_to_filename=MP4_PATH, fps=20)
    print(f"[saved] {MP4_PATH}")
    for nm, img in keyframes.items():
        Image.fromarray(img).save(os.path.join(OUT_DIR, f"Crushing_{nm}.png"))
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=True)

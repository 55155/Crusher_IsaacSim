"""
ipc_grasp_bag_test.py — 최소 격리 테스트 (IPC 커플러 + FEM cloth 샘플백).

목적
----
Crushing.py 전체를 FEM+IPC 로 포팅하기 전, 다음을 *격리*해서 검증한다:
  1. GPU(gs.cuda) + IPCCouplerOptions 에서 M0609+RG2 (articulated rigid) 와
     FEM.Cloth 봉투가 공존하며 crash(cudaErrorInvalidDevice) 없이 build·step 되는가.
  2. IPC 접촉: RG2 핑거를 닫을 때 rigid↔FEM-cloth 접촉으로 천이 눌리는가.
  3. 파지 유지: 참조 예제(examples/IPC_Solver/ipc_robot_grasp_cube.py) 패턴
       robot = Rigid(coup_type='two_way_soft_constraint',
                     coup_links=('rg2_left','rg2_right'), coup_friction=..)
     + FEM set_vertex_constraints(strip, link=finger) 로 봉투를 핑거에 구속 →
     top 구속 해제 후 arm 을 올리면 봉투가 그리퍼를 따라오는가.

PBD 버전(Crushing.py)과의 대응
------------------------------
  bag.fix_particles(idx)              → bag.set_vertex_constraints(idx)                (제자리 고정)
  bag.fix_particles_to_link(link,idx) → bag.set_vertex_constraints(idx, link=finger)   (링크 구속)
  bag.release_particle(idx)           → bag.remove_vertex_constraints(idx)
  bag.get_particles_pos()             → bag.get_state().pos

주의: IPC 는 PBD 를 못 본다(커플러가 fem_solver+rigid_solver 만 등록). 그래서 봉투는
      반드시 FEM.Cloth. contact_d_hat 은 봉투 두께(6mm)보다 작게(1mm) 잡아 초기 barrier
      위반을 피한다.
"""
import os, sys, shutil, tempfile
import xml.etree.ElementTree as ET
from datetime import datetime
import numpy as np
import trimesh as tm
from PIL import Image

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

# ── 봉투 형상 (Crushing.make_bag 와 동일 파라미터, 5-panel open pouch) ──────────
W, H, D = 0.05, 0.08, 0.006          # local: x=폭, y=높이, z=두께
NW, NH, ND = 6, 9, 2

# ── 시뮬 옵션 ────────────────────────────────────────────────────────────────
DT = 5e-3
IPC_D_HAT = 1.0e-3                    # 1mm < 봉투 두께 6mm → 초기 barrier off
RENDER_EVERY = 4

# ── FEM cloth 강성 (cloth teleop 예제 참고: 순수 IPC 접촉+마찰 파지) ──────────
CLOTH_E, CLOTH_NU, CLOTH_RHO = 6.0e4, 0.49, 200.0
CLOTH_THICK, CLOTH_BEND = 1.0e-3, 10.0
CLOTH_FRICTION = 0.5      # friction_mu (예제와 동일) — 접촉 마찰이 파지력의 핵심

# ── 로봇 자세 ────────────────────────────────────────────────────────────────
Q_GRASP = np.array([0, -0.40, 1.30, 0, 2.00, 0], float)
Q_LIFT  = np.array([0, -0.11, 0.60, 0, 2.41, 0], float)   # arm 올려 봉투 들어올림
FING_OPEN  = 0.040
FING_CLOSE = -0.020       # PD 목표를 관절범위 아래로 → 천을 강하게 squeeze (예제 -0.03 방식)

# 봉투 = 커플러 하 "실제" 핑거 패드 존 중심에 배치.
#   중요: two_way_soft_constraint 에서 팔이 set_dofs 목표(=probe FK)를 안 따라가고
#   커플러 아티큘레이션 솔버 기준 다른 자세로 정착 → probe FK(0.192,..,0.478) 무효.
#   빌드된 커플 씬에서 직접 측정한 pad center (결정적, strength 무관):
PAD_ZONE = np.array([0.2880, 0.0063, 0.5735])
BAG_POS  = tuple(PAD_ZONE)
BAG_EULER = (90, 0, 0)                # 두께(6mm)를 y(핑거 닫힘축)로, 높이(8cm)를 z 로

# ── 페이즈 스텝 ──────────────────────────────────────────────────────────────
N_SETTLE, N_CLOSE, N_GRASP, N_LIFT, N_HOLD = 100, 120, 60, 200, 80

# ── 경로 ────────────────────────────────────────────────────────────────────
_DIR = os.path.dirname(os.path.abspath(__file__))
_ROBOT_DIR = os.path.join(os.path.dirname(_DIR), "robots")   # Crusher_Genesis/robots
ROBOT_MJCF = os.path.join(_ROBOT_DIR, "m0609_rg2.xml")
OUT_DIR = os.path.join(os.path.dirname(_DIR), "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
STL_PATH = os.path.join(OUT_DIR, "_ipc_test_bag.stl")
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"ipc_grasp_bag_{_TS}.mp4")


def _npy(x): return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


# RG2 핑거/손은 원본 MJCF 에서 contype=0 → 충돌 geom 없음 → IPC 커플러가 핑거
# 접촉 형상을 못 받음(link.geoms 비어 있음). 런타임 패치로 contype/conaffinity 제거
# → default 0xFFFF(충돌 ON). (Crushing.patch_robot_mjcf 와 동일 취지)
RG2_GEOMS_TO_ENABLE = {"rg2_finger", "rg2_hand"}


def _prepare_robot_mjcf():
    """robots/ 전체 복사 + RG2 핑거 충돌 활성화 패치본 생성 → 패치 xml 경로 반환."""
    src_dir = _ROBOT_DIR
    tmp_dir = tempfile.mkdtemp(prefix="ipc_m0609_")
    for root_dir, _, files in os.walk(src_dir):
        rel = os.path.relpath(root_dir, src_dir)
        dst_dir = os.path.join(tmp_dir, rel) if rel != "." else tmp_dir
        os.makedirs(dst_dir, exist_ok=True)
        for f in files:
            shutil.copy2(os.path.join(root_dir, f), os.path.join(dst_dir, f))
    tree = ET.parse(ROBOT_MJCF); root = tree.getroot()
    wb = root.find("worldbody")
    n = 0
    if wb is not None:
        for g in wb.iter("geom"):
            if g.get("mesh") in RG2_GEOMS_TO_ENABLE:
                g.attrib.pop("contype", None)
                g.attrib.pop("conaffinity", None)
                n += 1
    dst = os.path.join(tmp_dir, "m0609_rg2_ipc.xml")
    tree.write(dst)
    print(f"[robot] RG2 collision enabled on {n} geoms → {dst}")
    return dst


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
    print(f"[bag] {len(m.vertices)} verts, {len(m.faces)} faces → {STL_PATH}")


def main(use_viewer: bool = False):
    print("="*60)
    print(f" IPC grasp bag test — M0609+RG2 + FEM cloth  (viewer={use_viewer})")
    print("="*60)
    make_bag()

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="info", precision="32")

    scene = gs.Scene(
        # 중력 OFF: 봉투를 핑거 높이에 띄워 두고 순수 IPC 접촉만 관찰
        # (얇은 봉투는 RG2 gap(~46mm) 로 pinch 불가 → vertex-pin 없이 접촉 변형만 검증)
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, 0)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=IPC_D_HAT,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,     # 로봇 self / plate 끼리 불필요
            enable_rigid_ground_contact=False,    # ground↔rigid 불필요 (봉투만 관심)
            constraint_strength_translation=100.0,   # ↑ 팔이 set_dofs 목표를 더 밀착 추종
            constraint_strength_rotation=100.0,
        ),
        vis_options=gs.options.VisOptions(background_color=(0.93, 0.94, 0.96)),
        show_viewer=use_viewer,
    )

    # ground (봉투가 떨어지면 받도록) — plane geom 은 IPC 에서 ipc_only 만 허용
    scene.add_entity(gs.morphs.Plane(),
                     material=gs.materials.Rigid(coup_type="ipc_only"))

    # 로봇: 핑거만 IPC 접촉 (two_way_soft_constraint). 핑거 충돌 활성화 패치본 사용.
    robot_xml = _prepare_robot_mjcf()
    robot = scene.add_entity(
        gs.morphs.MJCF(file=robot_xml, decimate=False),
        material=gs.materials.Rigid(
            coup_type="two_way_soft_constraint",
            coup_links=("rg2_left", "rg2_right"),
            coup_friction=CLOTH_FRICTION,
        ),
    )

    # FEM cloth 봉투
    bag = scene.add_entity(
        material=gs.materials.FEM.Cloth(
            E=CLOTH_E, nu=CLOTH_NU, rho=CLOTH_RHO,
            thickness=CLOTH_THICK, bending_stiffness=CLOTH_BEND,
            friction_mu=CLOTH_FRICTION,
        ),
        morph=gs.morphs.Mesh(file=STL_PATH, scale=1.0, pos=BAG_POS, euler=BAG_EULER),
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.75,
                                    roughness=0.9, double_sided=True),
    )

    cam = scene.add_camera(res=(960, 720), pos=(0.55, -0.55, 0.62),
                           lookat=(0.18, 0.0, 0.50), fov=42, GUI=False)

    scene.build(n_envs=0)

    left_link  = robot.get_link("rg2_left")
    right_link = robot.get_link("rg2_right")
    grip_link_idx = left_link.idx

    def _pad_center():
        """현재 상태에서 두 핑거 vgeom AABB 로 실제 '패드 존' 월드 중심 계산."""
        pts = []
        for L in (left_link, right_link):
            a = _npy(L.vgeoms[0].get_vAABB()).reshape(-1, 3)
            pts += [a.min(0), a.max(0)]
        pts = np.array(pts)
        return pts.mean(0)

    # ── 로봇 Q_GRASP(핑거 열림) 정착 후, 실제 pad center 와 봉투 위치 정렬 검증 ──
    #   (set_position 은 IPC-managed cloth 에 안 먹혀서 봉투는 빌드 시 BAG_POS=실측 pad
    #    center 로 이미 배치됨. 여기선 실제 정렬 오차만 확인.)
    for _ in range(40):
        robot.set_dofs_position(np.concatenate([Q_GRASP, [FING_OPEN, FING_OPEN]]))
        scene.step()
    pad_c = _pad_center()
    vp = _npy(bag.get_state().pos).squeeze()
    com_now = vp.mean(axis=0)
    print(f"[align] 실제 pad center={np.round(pad_c,4)}  bag com={np.round(com_now,4)}  "
          f"align err={np.linalg.norm(com_now-pad_c)*1e3:.1f}mm")
    print(f"[bag] verts z={vp[:,2].min():.3f}~{vp[:,2].max():.3f}  "
          f"y={vp[:,1].min():.3f}~{vp[:,1].max():.3f}")
    grip_idx = np.where(np.linalg.norm(vp - pad_c, axis=1) < 0.020)[0].astype(int)
    # NOTE: weld 없음. 예제(ipc_robot_cloth_teleop)처럼 핑거 close → IPC 접촉+마찰로만 파지.

    # 핑거는 PD 힘 제어로 세게 squeeze (set_dofs_position 은 soft-constraint 하에서
    # 천을 못 누름 → 예제처럼 kp 큰 control_dofs_position + 강한 close 명령).
    ARM_DOFS  = list(range(6))
    FING_DOFS = [6, 7]
    robot.set_dofs_kp(np.array([500.0, 500.0]), dofs_idx_local=FING_DOFS)
    robot.set_dofs_kv(np.array([50.0, 50.0]),  dofs_idx_local=FING_DOFS)

    frames = []
    cam.start_recording()

    def _shot(name):
        out = cam.render()
        rgb = out[0] if isinstance(out, (tuple, list)) else out   # (rgb, depth, seg, normal)
        rgb = _npy(rgb)
        rgb = rgb[0] if rgb.ndim == 4 else rgb
        Image.fromarray(rgb[..., :3].astype("uint8")).save(
            os.path.join(OUT_DIR, f"ipc_grasp_bag_{name}.png"))

    def _bag_com(): return _npy(bag.get_state().pos).squeeze().mean(axis=0)
    def _finger():  return _npy(left_link.get_pos()).squeeze()

    def run(name, q0, q1, f0, f1, n):
        for k in range(n):
            s = (k + 1) / n
            q = q0 + (q1 - q0) * s
            f = f0 + (f1 - f0) * s
            robot.set_dofs_position(q, dofs_idx_local=ARM_DOFS)          # 팔: kinematic 목표
            robot.control_dofs_position(np.array([f, f]), dofs_idx_local=FING_DOFS)  # 핑거: PD squeeze
            scene.step()
            if k % RENDER_EVERY == 0:
                cam.render()
        _shot(name)
        bc, fp = _bag_com(), _finger()
        print(f"[phase] {name:8s} @done  bag_com=({bc[0]:.3f},{bc[1]:.3f},{bc[2]:.3f})  "
              f"finger=({fp[0]:.3f},{fp[1]:.3f},{fp[2]:.3f})")

    com0 = _bag_com()
    # 1) settle: 봉투 띄운 채 안정화 (핑거 열림)
    run("settle", Q_GRASP, Q_GRASP, FING_OPEN, FING_OPEN, N_SETTLE)
    # 2) close: 핑거 꽉 닫음 → IPC 접촉+마찰로 천 pinch (weld 없음, 예제와 동일 방식)
    run("close",  Q_GRASP, Q_GRASP, FING_OPEN, FING_CLOSE, N_CLOSE)
    com_grasp = _bag_com(); fing_grasp = _finger()
    max_disp = float(np.linalg.norm(_npy(bag.get_state().pos).squeeze() - vp, axis=1).max())
    print(f"\n[contact-check] close 후 max vertex disp = {max_disp*1e3:.2f} mm  "
          f"→ {'CONTACT' if max_disp>1e-4 else 'NO contact'}")
    # 3) lift: arm 올림 → 봉투가 그리퍼를 따라오면 파지 성립
    run("lift",   Q_GRASP, Q_LIFT,  FING_CLOSE, FING_CLOSE, N_LIFT)
    run("hold",   Q_LIFT,  Q_LIFT,  FING_CLOSE, FING_CLOSE, N_HOLD)

    # ── 파지 판정: lift 동안 봉투 com 변위 vs 핑거 변위 (비율≈1 이면 따라온 것) ──
    com_end, fing_end = _bag_com(), _finger()
    d_bag  = com_end  - com_grasp
    d_fing = fing_end - fing_grasp
    ratio  = (np.linalg.norm(d_bag) / np.linalg.norm(d_fing)) if np.linalg.norm(d_fing) > 1e-6 else 0.0
    print(f"\n[grasp-check] lift 동안 finger Δ={np.round(d_fing*1e3,1)}mm (|{np.linalg.norm(d_fing)*1e3:.1f}|)")
    print(f"[grasp-check]              bag Δ={np.round(d_bag*1e3,1)}mm (|{np.linalg.norm(d_bag)*1e3:.1f}|)")
    print(f"[grasp-check] follow ratio = {ratio:.2f}  → "
          f"{'GRASP HELD (봉투가 그리퍼 따라옴)' if ratio > 0.6 else 'SLIPPED/DROPPED (파지 실패)'}")

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved video] {MP4}")
    print("[done] phase PNGs → Sim_result/ipc_grasp_bag_<phase>.png")


if __name__ == "__main__":
    main(use_viewer=False)

"""
ipc_grasp_bag_test.py — 최소 격리 테스트 (IPC 커플러 + FEM cloth 샘플백).

목적
----
Crushing.py 전체를 FEM+IPC 로 포팅하기 전, 다음을 *격리*해서 검증한다:
  1. GPU(gs.cuda) + IPCCouplerOptions 에서 M0609+RG2 (articulated rigid) 와
     FEM.Cloth 봉투가 공존하며 crash(cudaErrorInvalidDevice) 없이 build·step 되는가.
  2. IPC 접촉: RG2 핑거를 닫을 때 rigid↔FEM-cloth 접촉으로 천이 눌리는가.
  3. 마찰 파지: 중력 ON 에서 봉투를 선반(shelf) 위에 두고, 핑거를 pad-gap 3mm
     까지 닫아(봉투 6mm 압착) arm 을 올린다. 봉투가 선반을 떠나 그리퍼를
     따라오면 성공 — vertex-pin 없는 순수 IPC contact+friction 파지.

핵심 발견 (2026-07-08)
----------------------
  · IPC 커플러에는 FEM vertex constraint 경로가 없다 (1.1.0):
    set_vertex_constraints 는 fem_entity.py 에서 IPC 시 예외,
    _add_fem_entities_to_ipc() 도 재질만 등록 (SoftPositionConstraint 미배선).
    → PBD 의 fix_particles_to_link 등가물이 없어 "마찰 파지"가 유일한 길.
  · RG2 "34mm 최소 gap" 은 링크 원점 간 거리였고, finger.obj 패드 안쪽 면은
    원점에서 안으로 뻗어 있어 실제 pad-gap = 1mm + 2q. q=0.001 → 3mm.
    6mm 봉투 압착 가능. (기존 FING_CLOSE=0.006 은 gap 13mm → 스침만 발생)

주의: IPC 는 PBD 를 못 본다(커플러가 fem_solver+rigid_solver 만 등록). 그래서 봉투는
      반드시 FEM.Cloth. contact_d_hat 은 봉투 두께(6mm)보다 작게(1mm) 잡아 초기 barrier
      위반을 피한다. IPC world 의 중력은 build 시 sim_options 에서 박제되어
      런타임 set_gravity 로 못 바꾼다 (ipc_coupler/utils.py).
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

# ── FEM cloth 강성 (grasp 예제 참고: E=1e5, nu=0.499, thin shell) ─────────────
CLOTH_E, CLOTH_NU, CLOTH_RHO = 1.0e5, 0.499, 200.0
CLOTH_THICK, CLOTH_BEND = 1.0e-3, 50.0
CLOTH_FRICTION = 0.8

# ── 로봇 자세 (FK probe 로 확인: 이 자세에서 핑거 중점 ≈ (0.18,0.006,0.52)) ──
Q_GRASP = np.array([0, -0.40, 1.30, 0, 2.00, 0], float)
Q_LIFT  = np.array([0, -0.11, 0.60, 0, 2.41, 0], float)   # arm 을 올려 봉투 들어올림
# pad-gap = 1mm + 2q (finger.obj 팁 안쪽면 실측). q=0.001 → 3mm < 봉투 6mm = 압착
FING_OPEN, FING_CLOSE = 0.040, 0.001

# 봉투는 "스텝 중 평형" 패드 중점에 맞춰 배치 (FK_PROBE=1 실측, 2026-07-08).
# 주의: IPC two_way_soft_constraint 는 양방향이라 teleport 자세를 IPC 가 되밀어
#       순수 FK 와 다른 평형 자세에 정착한다 (링크 원점 0.522 → 0.608, +8.5cm).
#       반드시 스텝 유지 상태의 실측값을 쓴다.
FINGER_MID = np.array([0.3239, 0.0063, 0.5283])
BAG_POS    = tuple(FINGER_MID)        # 봉투 중심 = 패드 중점 (실측 평형)
BAG_EULER  = (90, 0, 0)               # 두께(6mm)를 y(핑거 닫힘축)로, 높이(8cm)를 z 로

# 봉투 받침 선반: 봉투 바닥(중심-4cm) 1.5mm 아래에 top. settle 동안 봉투가 얹혀
# 낙하를 막고, lift 시 봉투가 선반을 떠나는지가 파지 성공의 자명한 판정이 된다.
SHELF_TOP  = float(FINGER_MID[2]) - 0.04 - 0.0015
SHELF_SIZE = (0.12, 0.12, 0.02)
SHELF_POS  = (float(FINGER_MID[0]), float(FINGER_MID[1]), SHELF_TOP - SHELF_SIZE[2] / 2)

# ── 페이즈 스텝 ──────────────────────────────────────────────────────────────
N_SETTLE, N_CLOSE, N_GRASP, N_LIFT, N_HOLD = 120, 80, 40, 200, 100

# ── 경로: config.json + paths.py (중앙 해석) — 위치 독립 부트스트랩 ──────────
_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
import paths

ROBOT_MJCF = paths.ROBOT_M0609_RG2
OUT_DIR = paths.SIM_RESULT
STL_PATH = os.path.join(OUT_DIR, "_ipc_test_bag.stl")
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"ipc_grasp_bag_{_TS}.mp4")


def _npy(x): return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def _panel(fn, nu, nv):
    t = []
    for i in range(nu):
        for j in range(nv):
            a, b = fn(i/nu, j/nv), fn((i+1)/nu, j/nv)
            c, d = fn((i+1)/nu, (j+1)/nv), fn(i/nu, (j+1)/nv)
            t += [[a, b, c], [a, c, d]]
    return t


def _prepare_robot_mjcf():
    """m0609_rg2.xml 사본 + RG2 핑거 geom 의 contype=0/conaffinity=0 제거.

    원본 XML 은 핑거 mesh 가 visual-only 라 Genesis 가 collision geom 을 만들지
    않고, IPC 커플러도 link.geoms 만 등록한다 → 원본 그대로면 핑거가 IPC 에
    존재하지 않아 cloth 와 접촉 자체가 불가능 (2026-07-08 GRASP FAIL 원인).
    Crushing.py 의 patch_robot_mjcf 와 동일 패턴.
    """
    src_dir = os.path.dirname(ROBOT_MJCF)
    tmp_dir = tempfile.mkdtemp(prefix="m0609_ipc_")
    for root_dir, _, files in os.walk(src_dir):
        rel = os.path.relpath(root_dir, src_dir)
        dst_dir = os.path.join(tmp_dir, rel) if rel != "." else tmp_dir
        os.makedirs(dst_dir, exist_ok=True)
        for f in files:
            shutil.copy2(os.path.join(root_dir, f), os.path.join(dst_dir, f))
    dst = os.path.join(tmp_dir, "m0609_rg2_patched.xml")
    tree = ET.parse(ROBOT_MJCF); root = tree.getroot()
    wb = root.find("worldbody")
    if wb is not None:
        for g in wb.iter("geom"):
            if g.get("mesh") == "rg2_finger":
                g.attrib.pop("contype", None)
                g.attrib.pop("conaffinity", None)
    tree.write(dst)
    return dst


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
        # 중력 ON: 봉투는 선반 위에서 settle → pinch → lift. IPC world 중력은
        # build 시 박제라 런타임 토글 불가 → 처음부터 켠다.
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=IPC_D_HAT,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,     # 로봇 self / plate 끼리 불필요
            enable_rigid_ground_contact=False,    # ground↔rigid 불필요 (봉투만 관심)
            # 기본값 100. 10 으로 낮추면 IPC ABD 핑거가 aim(gs 자세)을 못 따라가
            # ~8.5cm 처진 평형 → 렌더(gs)와 접촉(IPC)이 서로 다른 곳에 있게 됨.
            constraint_strength_translation=100.0,
            constraint_strength_rotation=100.0,
        ),
        vis_options=gs.options.VisOptions(background_color=(0.93, 0.94, 0.96)),
        show_viewer=use_viewer,
    )

    # ground (봉투가 떨어지면 받도록) — plane geom 은 IPC 에서 ipc_only 만 허용
    scene.add_entity(gs.morphs.Plane(),
                     material=gs.materials.Rigid(coup_type="ipc_only"))

    # 봉투 받침 선반 (정적 rigid, IPC 접촉만)
    scene.add_entity(
        gs.morphs.Box(size=SHELF_SIZE, pos=SHELF_POS, fixed=True),
        material=gs.materials.Rigid(coup_type="ipc_only", coup_friction=0.3),
        surface=gs.surfaces.Default(color=(0.75, 0.78, 0.82)),
    )

    # 로봇: 핑거만 IPC 접촉 (two_way_soft_constraint)
    #   원본 XML 은 핑거가 visual-only → 패치본으로 collision geom 생성 필수
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

    # ── 시작 자세 (핑거 열림) ──
    robot.set_dofs_position(np.concatenate([Q_GRASP, [FING_OPEN, FING_OPEN]]))
    left_link = robot.get_link("rg2_left")
    grip_link_idx = left_link.idx

    # ── 봉투 정점 위치로 top edge / grip strip 선정 ──
    vp = _npy(bag.get_state().pos).squeeze()
    print(f"[bag] built verts={vp.shape}  z={vp[:,2].min():.3f}~{vp[:,2].max():.3f}  "
          f"x={vp[:,0].min():.3f}~{vp[:,0].max():.3f}  y={vp[:,1].min():.3f}~{vp[:,1].max():.3f}")
    # grip strip: 핑거 중점 근처 (파지 추적용 진단 인덱스)
    d_to_mid = np.linalg.norm(vp - FINGER_MID, axis=1)
    grip_idx = np.where(d_to_mid < 0.020)[0].astype(int)
    print(f"[bag] grip_strip verts near finger_mid={len(grip_idx)}")
    # NOTE: IPC 커플러엔 FEM vertex constraint 가 없다 (헤더 참조) →
    #       vertex-pin 없이 순수 contact+friction 파지로 검증한다.

    # ── FK probe 모드: Q_GRASP 에서 실제 패드 중심 월드좌표 실측 (FK_PROBE=1) ──
    #    pad 중심 (link local): finger.obj 팁 x=80~122mm, 안쪽면 y=-16.5~-5mm 실측
    #    → left (0.101, -0.0107, 0) / right 는 geom euler 180°x 반전 → (0.101, +0.0107, 0)
    if os.environ.get("FK_PROBE") == "1":
        import genesis.utils.geom as gu

        def _pads(tag):
            pads = {}
            for nm, loc in (("rg2_left", (0.101, -0.0107, 0.0)),
                            ("rg2_right", (0.101, +0.0107, 0.0))):
                lk = robot.get_link(nm)
                p = _npy(lk.get_pos()).squeeze(); q = _npy(lk.get_quat()).squeeze()
                T = np.asarray(gu.trans_quat_to_T(p, q))
                pads[nm] = T[:3, :3] @ np.asarray(loc) + T[:3, 3]
                ax = T[:3, :3] @ np.array([0.0, 1.0, 0.0])   # 닫힘축(로컬 y)
                print(f"[probe:{tag}] {nm}: origin={np.round(p,4)} pad={np.round(pads[nm],4)} "
                      f"close_axis={np.round(ax,3)}")
            mid = (pads["rg2_left"] + pads["rg2_right"]) / 2
            print(f"[probe:{tag}] pad_mid = {np.round(mid, 4)}")
            return mid

        robot.set_dofs_position(np.concatenate([Q_GRASP, [FING_OPEN, FING_OPEN]]))
        _pads("set0")                       # 스텝 없이 순수 FK
        for _ in range(5):
            robot.set_dofs_position(np.concatenate([Q_GRASP, [FING_OPEN, FING_OPEN]]))
            scene.step()
        mid = _pads("step5")                # 매 스텝 teleport 유지 후 (본 시퀀스와 동일)
        print(f"[probe] 현재 FINGER_MID={FINGER_MID}")
        out = cam.render()
        rgb = out[0] if isinstance(out, (tuple, list)) else out
        rgb = _npy(rgb); rgb = rgb[0] if rgb.ndim == 4 else rgb
        Image.fromarray(rgb[..., :3].astype("uint8")).save(
            os.path.join(OUT_DIR, "ipc_grasp_bag_probe.png"))
        return

    frames = []
    cam.start_recording()

    def _shot(name):
        out = cam.render()
        rgb = out[0] if isinstance(out, (tuple, list)) else out   # (rgb, depth, seg, normal)
        rgb = _npy(rgb)
        rgb = rgb[0] if rgb.ndim == 4 else rgb
        Image.fromarray(rgb[..., :3].astype("uint8")).save(
            os.path.join(OUT_DIR, f"ipc_grasp_bag_{name}.png"))

    def _finger_z():
        p = _npy(left_link.get_pos()).squeeze()
        return float(p[2])

    def run(name, q0, q1, f0, f1, n):
        for k in range(n):
            s = (k + 1) / n
            q = q0 + (q1 - q0) * s
            f = f0 + (f1 - f0) * s
            robot.set_dofs_position(np.concatenate([q, [f, f]]))
            scene.step()
            if k % RENDER_EVERY == 0:
                cam.render()
        _shot(name)
        vpn = _npy(bag.get_state().pos).squeeze()
        print(f"[phase] {name:8s} @done  bag_com=({vpn[:,0].mean():.3f},"
              f"{vpn[:,1].mean():.3f},{vpn[:,2].mean():.3f})  "
              f"grip_com_z={vpn[grip_idx,2].mean():.3f}  finger_z={_finger_z():.3f}")
        return vpn

    # 1) settle: 봉투가 선반 위에 얹혀 안정화 (핑거 열림)
    run("settle", Q_GRASP, Q_GRASP, FING_OPEN, FING_OPEN, N_SETTLE)
    # 2) close: 핑거 pad-gap 3mm 까지 닫아 봉투 압착
    run("close",  Q_GRASP, Q_GRASP, FING_OPEN, FING_CLOSE, N_CLOSE)
    # 3) grasp: 압착 유지 안정화
    vp_pre = run("grasp", Q_GRASP, Q_GRASP, FING_CLOSE, FING_CLOSE, N_GRASP)
    fz_pre = _finger_z()
    # 4) lift: arm 상승 — 봉투가 선반을 떠나 따라오는가?
    run("lift",   Q_GRASP, Q_LIFT, FING_CLOSE, FING_CLOSE, N_LIFT)
    # 5) hold: 들린 채 유지 (slip 관찰)
    vp_post = run("hold", Q_LIFT, Q_LIFT, FING_CLOSE, FING_CLOSE, N_HOLD)
    fz_post = _finger_z()

    # 파지 판정: lift 동안 핑거 상승분 vs 봉투 grip strip 상승분
    dz_f = fz_post - fz_pre
    dz_b = float(vp_post[grip_idx, 2].mean() - vp_pre[grip_idx, 2].mean())
    slip = dz_f - dz_b
    ok = dz_f > 0.02 and dz_b > 0.5 * dz_f
    print(f"\n[grasp-check] finger Δz = {dz_f*1e3:+.1f} mm,  bag grip Δz = {dz_b*1e3:+.1f} mm,"
          f"  slip = {slip*1e3:+.1f} mm")
    print(f"[grasp-check] → {'GRASP OK (bag follows gripper)' if ok else 'GRASP FAIL (bag left behind / slipped)'}")

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved video] {MP4}")
    print("[done] phase PNGs → Sim_result/ipc_grasp_bag_<phase>.png")


if __name__ == "__main__":
    main(use_viewer=False)

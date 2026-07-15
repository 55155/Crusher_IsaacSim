"""
tablet_bag_grasp_pipeline.py — 통합 파이프라인: 정제(FEM.Elastic) 낙하 →
샘플백(FEM.Cloth)이 받음 → M0609+RG2(v2, mimic joint + convex decomposition)
가 봉투 실링부를 마찰 파지·리프트.

조합 (§docs/DigitalTwin.md §9 "Solver / Coupler 조합 실험 기록" 참고):
  - Tablet: **FEM.Elastic, sliver-free medial-axis 캡슐**(§9 조합 5).
  - Samplebag: **FEM.Cloth**(2D 표면 전용). §9 조합 2/5 검증값 그대로.
  - Robotarm: **Rigid, M0609+RG2 v2**(mimic joint + CoACD convex
    decomposition 손가락). §9 조합 2 검증값 그대로.
  - Coupler: **IPC**, 전부 관통.

**왜 Rigid+SDF(§9 조합 6) 대신 FEM인가 — 우회 배경(2026-07-15)**:
  처음엔 정제를 Rigid+SDF(MJCF capsule geom)로 넣었다(조합 6이 단독으로는
  검증됐으므로). 하지만 **로봇(Rigid, two_way_soft_constraint)까지 같은 씬에
  들어가면** `constraint_strength` 를 100(로봇 단독 검증값)이든 10/30
  이든 몇 스텝 안에 지수적으로 발산했고, 1.0으로 낮추면 발산은 막히지만
  이번엔 정제가 봉투를 그대로 뚫고 지나가 버렸다(접촉 해석 자체가 무력화).
  Genesis 공식 예제(`examples/IPC_Solver/ipc_robot_cloth_teleop.py`)에도
  Rigid(로봇, two_way_soft_constraint) + FEM.Cloth + Rigid(ipc_only) 조합이
  있지만, 그 `ipc_only` 엔티티들은 전부 `fixed=True`(정적 소품)였다 — **동적
  (freejoint) `ipc_only` 강체가 `two_way_soft_constraint` 강체와 공존하는
  조합은 공식 예제에도 없다.** 즉 우리가 처음 겪은 사례는 Genesis IPC
  커플러가 검증하지 않은 조합으로 판단, 정제를 **FEM(§9 조합 5, 로봇 앞에서도
  이미 안정성 확인된 sliver-free 캡슐)으로 우회**한다 — Rigid 엔티티는
  로봇 하나뿐이라 이 mismatch 자체가 사라진다.

핵심 설정(§9 조합 2/5 그대로):
  - `fem_options` 를 일부러 안 준다(implicit 로 켜면 Rigid 존재 시 FEM 중력
    적분이 멈추는 버그가 있음).
  - `contact_d_hat` 은 캡슐 극(pole) 근처 정점 간격보다 작게(§9 조합 5 교훈,
    self-contact 오탐 방지) — `make_capsule_tets_v2` 출력 기준 1e-4 사용.
  - 로봇 `constraint_strength` 는 100.0(§9 조합 2 원래 검증값) 그대로 — Rigid
    쪽에 이제 로봇 하나뿐이라 완화할 필요가 없다.

출력: RESULT/tablet_bag_grasp_<ts>.mp4 + phase PNG들.
"""
import os, sys, shutil, tempfile
import xml.etree.ElementTree as ET
from datetime import datetime
import numpy as np
from PIL import Image

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
import paths

sys.path.insert(0, os.path.join(os.path.dirname(_r), "utills"))
from primitive_tablet_generator import make_capsule_tets_v2, add_analytic_fem_entity

ROBOT_MJCF = os.path.join(paths.ROBOTS_DIR, "m0609_rg2_v2.xml")
COACD_DIR_REL = "rg2/reference_onrobot_ros/meshes/rg2_v1/coacd"
FLEX_FINGER_HULLS = [f"flex_finger_hull_{i:03d}.stl" for i in range(7)]
BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag", "Samplebag_seal_pouch3.stl")

# ── 정제 — FEM.Elastic, sliver-free 캡슐(§9 조합 5). 납작한 원반형(실제 정제
# 비율에 가까움) — 지름(4mm) < 봉투 두께(6mm)로 여유를 둬 입구 통과 가능케 함.
CAP_RADIUS_MM, CAP_CYL_H_MM = 2.0, 1.0
TABLET_E, TABLET_NU, TABLET_RHO = 5.0e4, 0.45, 1300.0
TABLET_FRICTION = 0.5

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"tablet_bag_grasp_{_TS}.mp4")

# ── 시뮬 옵션 ────────────────────────────────────────────────────────────────
DT = 5e-3
# 캡슐 극 근처 정점 간격보다 작게(§9 조합5 self-contact 오탐 방지) — 봉투
# 두께(6mm) 대비 정제 지름(4mm) 통과 여지 확보에도 도움.
IPC_D_HAT = 1.0e-4
RENDER_EVERY = 4

# ── FEM cloth(봉투) — grasp_bag_ipc_test.py 검증값 ─────────────────────────
CLOTH_E, CLOTH_NU, CLOTH_RHO = 1.0e5, 0.499, 200.0
CLOTH_THICK, CLOTH_BEND = 1.0e-3, 50.0
CLOTH_FRICTION = 0.8

FINGER_LINKS = ("f1_flex_finger", "f2_flex_finger")
Q_GRASP = np.array([0, -0.40, 1.30, 0, 2.00, np.pi / 2], float)
Q_LIFT = np.array([0, -0.11, 0.60, 0, 2.41, np.pi / 2], float)
FING_OPEN, FING_CLOSE = 1.00, 1.20
FINGER_MID = np.array([0.20365, 0.00618, 0.43297])

# ── 봉투 배치 (grasp_bag_ipc_test.py 와 동일) ───────────────────────────────
BAG_SCALE = 1.0
BAG_EULER = (90, 0, 0)
SEAL_LOCAL_X = -0.028
BAG_POS = (FINGER_MID[0] - SEAL_LOCAL_X, FINGER_MID[1], FINGER_MID[2])
BAG_HALF_H = 0.045  # 프록시 패널 높이(90mm) 절반 — 로컬 y 범위 ±45mm

SHELF_TOP = BAG_POS[2] - BAG_HALF_H - 0.0015
SHELF_SIZE = (0.10, 0.10, 0.02)
SHELF_POS = (BAG_POS[0], BAG_POS[1], SHELF_TOP - SHELF_SIZE[2] / 2)

# ── 정제 낙하 위치: 봉투 입구(로컬 y=+45mm → world Z 최상단) 바로 위 ────────
BAG_MOUTH_Z = BAG_POS[2] + BAG_HALF_H
TABLET_DROP_H = 0.015   # 입구 위 15mm 에서 낙하
TABLET_POS = (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z + TABLET_DROP_H)

N_DROP, N_SETTLE, N_CLOSE, N_GRASP, N_LIFT, N_HOLD = 150, 60, 80, 40, 200, 100

CAM_POS, CAM_LOOK = (0.75, -0.55, 0.70), tuple(FINGER_MID + np.array([0, 0, 0.03]))


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def _prepare_robot_mjcf():
    """grasp_bag_ipc_test.py::_prepare_robot_mjcf 와 동일 — mimic joint(1개
    능동 관절 gripper_joint + <equality><joint> 1:1 동기화 5개)를 Genesis 가
    존중 안 하므로, 실제 구동은 6개 DOF를 매 스텝 동일 값으로 set_dofs_position
    하는 방식으로 대체(§9 Robotarm 공통 테크닉). 손가락은 CoACD 볼록분해
    7조각을 collision geom 으로 주입.
    """
    src_dir = paths.ROBOTS_DIR
    tmp_dir = tempfile.mkdtemp(prefix="m0609_tbs_v2_")
    for root_dir, _, files in os.walk(src_dir):
        rel = os.path.relpath(root_dir, src_dir)
        dst_dir = os.path.join(tmp_dir, rel) if rel != "." else tmp_dir
        os.makedirs(dst_dir, exist_ok=True)
        for f in files:
            shutil.copy2(os.path.join(root_dir, f), os.path.join(dst_dir, f))
    dst = os.path.join(tmp_dir, "m0609_rg2_v2_patched.xml")
    tree = ET.parse(ROBOT_MJCF)
    root = tree.getroot()

    asset = root.find("asset")
    for i, hull_file in enumerate(FLEX_FINGER_HULLS):
        mesh_el = ET.SubElement(asset, "mesh")
        mesh_el.set("name", f"flex_finger_hull_{i:03d}")
        mesh_el.set("file", f"{COACD_DIR_REL}/{hull_file}")

    wb = root.find("worldbody")
    for j in wb.iter("joint"):
        j.attrib.pop("damping", None)
        j.attrib.pop("frictionloss", None)
    for body in wb.iter("body"):
        if body.get("name") in ("f1_flex_finger", "f2_flex_finger"):
            for i in range(len(FLEX_FINGER_HULLS)):
                g = ET.SubElement(body, "geom")
                g.set("type", "mesh")
                g.set("mesh", f"flex_finger_hull_{i:03d}")
                g.set("contype", "1")
                g.set("conaffinity", "1")
                g.set("group", "0")
                g.set("friction", "1.5 0.02 0.001")

    for tag in ("actuator", "equality"):
        el = root.find(tag)
        if el is not None:
            root.remove(el)
    tree.write(dst)
    return dst


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" Tablet(FEM sliver-free) drop -> Bag(FEM) catch -> M0609+RG2(mimic+CoACD) grasp/lift (viewer={use_viewer})")
    print("=" * 60)

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        # fem_options 를 일부러 안 준다 — implicit 로 켜면 Rigid 엔티티 존재 시
        # FEM 이 중력 적분을 멈추는 버그가 있다(§8/§9 조합5).
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=IPC_D_HAT,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=False,
            # Rigid 는 로봇 하나뿐(정제가 FEM 이라 ipc_only 강체 mismatch가
            # 없음) — §9 조합2 원래 검증값 100.0 그대로 사용.
            constraint_strength_translation=100.0,
            constraint_strength_rotation=100.0,
        ),
        vis_options=gs.options.VisOptions(background_color=(0.93, 0.94, 0.96)),
        show_viewer=use_viewer,
    )

    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(coup_type="ipc_only"))

    scene.add_entity(
        gs.morphs.Box(size=SHELF_SIZE, pos=SHELF_POS, fixed=True),
        material=gs.materials.Rigid(coup_type="ipc_only", coup_friction=0.3),
        surface=gs.surfaces.Default(color=(0.75, 0.78, 0.82)),
    )

    robot_xml = _prepare_robot_mjcf()
    robot = scene.add_entity(
        gs.morphs.MJCF(file=robot_xml, decimate=False),
        material=gs.materials.Rigid(
            coup_type="two_way_soft_constraint",
            coup_links=FINGER_LINKS,
            coup_friction=CLOTH_FRICTION,
        ),
    )

    bag = scene.add_entity(
        material=gs.materials.FEM.Cloth(
            E=CLOTH_E, nu=CLOTH_NU, rho=CLOTH_RHO,
            thickness=CLOTH_THICK, bending_stiffness=CLOTH_BEND,
            friction_mu=CLOTH_FRICTION,
        ),
        morph=gs.morphs.Mesh(file=BAG_STL, scale=BAG_SCALE, pos=BAG_POS, euler=BAG_EULER),
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.55,
                                     roughness=0.9, double_sided=True),
    )

    cap_verts_mm, cap_elems = make_capsule_tets_v2(
        radius_mm=CAP_RADIUS_MM, cyl_height_mm=CAP_CYL_H_MM, n_theta=12, n_cap_rings=4, n_cyl_bands=2,
    )
    print(f"[tablet] capsule verts={len(cap_verts_mm)} tets={len(cap_elems)} (sliver-free medial-axis, TetGen 미사용)")

    tablet = add_analytic_fem_entity(
        scene, key=os.path.join(OUT_DIR, "_analytic_capsule_v2.stl"),
        verts_mm=cap_verts_mm, elems=cap_elems,
        material=gs.materials.FEM.Elastic(
            E=TABLET_E, nu=TABLET_NU, rho=TABLET_RHO,
            friction_mu=TABLET_FRICTION, model="stable_neohookean",
        ),
        scale=1e-3, pos=TABLET_POS,
        surface=gs.surfaces.Default(color=(0.9, 0.9, 0.85), roughness=0.6),
    )

    cam = scene.add_camera(res=(960, 720), pos=CAM_POS, lookat=CAM_LOOK, fov=45, GUI=False)
    scene.build(n_envs=0)

    q_grasp, q_lift = Q_GRASP, Q_LIFT
    robot.set_dofs_position(np.concatenate([q_grasp, [FING_OPEN] * 6]))
    left_link = robot.get_link(FINGER_LINKS[0])

    vp0 = _npy(bag.get_state().pos).squeeze()
    print(f"[bag]    verts={vp0.shape}  x={vp0[:,0].min():.4f}~{vp0[:,0].max():.4f}  "
          f"y={vp0[:,1].min():.4f}~{vp0[:,1].max():.4f}  z={vp0[:,2].min():.4f}~{vp0[:,2].max():.4f}")
    d_to_mid = np.linalg.norm(vp0 - FINGER_MID, axis=1)
    grip_idx = np.where(d_to_mid < 0.020)[0].astype(int)
    print(f"[bag]    grip_strip verts near FINGER_MID: {len(grip_idx)}")

    tp0 = _npy(tablet.get_state().pos).squeeze()
    print(f"[tablet] verts={tp0.shape}  z0_mean={tp0[:,2].mean()*1e3:.2f}mm  "
          f"bag_mouth_z={BAG_MOUTH_Z*1e3:.2f}mm")

    cam.start_recording()

    def _shot(name):
        out = cam.render()
        rgb = out[0] if isinstance(out, (tuple, list)) else out
        rgb = _npy(rgb)
        rgb = rgb[0] if rgb.ndim == 4 else rgb
        Image.fromarray(rgb[..., :3].astype("uint8")).save(
            os.path.join(OUT_DIR, f"tablet_bag_grasp_{name}.png"))

    def _finger_z():
        return float(_npy(left_link.get_pos()).squeeze()[2])

    def _tablet_z():
        return float(_npy(tablet.get_state().pos).squeeze()[:, 2].mean())

    def run(name, q0, q1, f0, f1, n, trace=False):
        diverged = False
        for k in range(n):
            s = (k + 1) / n
            q = q0 + (q1 - q0) * s
            f = f0 + (f1 - f0) * s
            robot.set_dofs_position(np.concatenate([q, [f] * 6]))
            scene.step()
            if trace and k % 20 == 0:
                print(f"    [{name} k={k:4d}] tablet_z={_tablet_z()*1e3:+.2f}mm")
            if k % RENDER_EVERY == 0:
                cam.render()
            if abs(_tablet_z()) > 2.0:
                diverged = True
                print(f"  [!!] 정제 발산 감지(|z|>2m) — 조기 종료")
                break
        _shot(name)
        vpn = _npy(bag.get_state().pos).squeeze()
        gz = vpn[grip_idx, 2].mean() if len(grip_idx) else float("nan")
        print(f"[phase] {name:8s} @done  bag_com=({vpn[:,0].mean():.3f},{vpn[:,1].mean():.3f},"
              f"{vpn[:,2].mean():.3f})  grip_com_z={gz:.4f}  finger_z={_finger_z():.4f}  "
              f"tablet_z={_tablet_z()*1e3:+.2f}mm")
        return vpn, diverged

    # 1) drop: 그리퍼는 대기(open), 정제가 중력으로 낙하해 봉투 입구로 들어감
    _, div = run("drop", q_grasp, q_grasp, FING_OPEN, FING_OPEN, N_DROP, trace=True)
    if div:
        cam.stop_recording(save_to_filename=MP4, fps=30)
        print(f"\n[aborted] 발산으로 조기 종료, 영상 저장: {MP4}")
        return
    # 2) settle: 정제가 봉투 안에서 안정화
    run("settle", q_grasp, q_grasp, FING_OPEN, FING_OPEN, N_SETTLE)
    # 3) close: 그리퍼가 실링부를 압착
    run("close", q_grasp, q_grasp, FING_OPEN, FING_CLOSE, N_CLOSE)
    vp_pre, _ = run("grasp", q_grasp, q_grasp, FING_CLOSE, FING_CLOSE, N_GRASP)
    fz_pre = _finger_z()
    tz_pre = _tablet_z()
    # 4) lift: 봉투(+정제) 를 함께 들어올림
    run("lift", q_grasp, q_lift, FING_CLOSE, FING_CLOSE, N_LIFT)
    vp_post, _ = run("hold", q_lift, q_lift, FING_CLOSE, FING_CLOSE, N_HOLD)
    fz_post = _finger_z()
    tz_post = _tablet_z()

    dz_f = fz_post - fz_pre
    dz_b = float(vp_post[grip_idx, 2].mean() - vp_pre[grip_idx, 2].mean()) if len(grip_idx) else float("nan")
    dz_t = tz_post - tz_pre
    print(f"\n[check] finger Δz={dz_f*1e3:+.1f}mm  bag grip Δz={dz_b*1e3:+.1f}mm  "
          f"tablet Δz={dz_t*1e3:+.1f}mm")
    bag_ok = len(grip_idx) > 0 and dz_f > 0.02 and dz_b > 0.5 * dz_f
    tablet_ok = dz_t > 0.5 * dz_f
    print(f"[check] 봉투 파지: {'OK' if bag_ok else 'FAIL'}  /  정제 동반 상승: "
          f"{'OK' if tablet_ok else 'FAIL(봉투 밖으로 빠졌을 수 있음)'}")

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved video] {MP4}")
    print("[done] phase PNGs -> RESULT/tablet_bag_grasp_<phase>.png")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

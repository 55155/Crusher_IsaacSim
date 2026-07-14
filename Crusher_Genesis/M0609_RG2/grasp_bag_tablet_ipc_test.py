"""
grasp_bag_tablet_ipc_test.py — 통합 파이프라인: 정제(FEM.Elastic) 낙하 →
샘플백(FEM.Cloth)이 받음 → M0609+RG2(v2) 가 봉투 실링부를 마찰 파지·리프트.

grasp_bag_ipc_test.py(봉투 파지, 검증됨)에 정제 낙하 단계를 추가한 것.

**핵심 발견(2026-07-14) — FEM.Elastic "정지 상태에서 중력 무시" 버그 해결**:
  전날 fem_tablet_drop.py 에서 발견한 "implicit FEM 솔버가 Rigid 엔티티 존재
  시 중력을 무시하고 얼어붙는" 문제는, **`fem_options=FEMOptions(use_implicit_
  solver=True)` 를 명시적으로 켠 것 자체가 원인**이었다. Genesis 공식 예제
  `examples/IPC_Solver/ipc_robot_grasp_cube.py` (Rigid Franka + FEM.Elastic
  큐브) 를 그대로 재현해보니 — **fem_options 를 아예 안 주는 기본값(non-
  implicit)** + 무른 재질(E=5e4, model="stable_neohookean") 조합이면 Rigid
  엔티티가 있어도 킥 없이 정상 낙하함을 격리 테스트로 확인. 이 스크립트도
  fem_options 를 건드리지 않는다(grasp_bag_ipc_test.py 도 원래 안 건드림 —
  봉투가 잘 떨어졌던 이유).
  단, 실측 캘리브레이션 값(E=2GPa)은 이 조합(explicit, dt=5e-3)에서 못 버티고
  발산한다 — 이 통합 테스트는 "낙하→포집→파지 파이프라인 검증"이 목적이라
  파괴역학 정밀도보다 안정성을 우선해 재질을 무르게(E=5e4) 낮춘다. 정밀 압축
  거동은 fem_uniaxial_compression.py(E=2GPa, quasi-static, SAP)가 별도로 맡는다.

배치: 봉투는 grasp_bag_ipc_test.py 와 동일하게 실링부(왼쪽)를 파지 타깃으로
두고 서 있으며, 입구(로컬 y=+45mm → world Z 최상단)가 위를 향한다. 정제는
그 입구 바로 위에서 낙하시켜 봉투 안에 담기게 한다.

**2번째 발견(2026-07-14) — M0609 로봇이 있으면 FEM.Elastic 이 여전히 얼어붙음,
단 "박스 프리미티브"는 예외**:
  위 픽스(fem_options 안 건드리기) 만으로는 부족했다 — M0609(v1/v2 무관, 팔
  collision 전부 꺼도, decimate/recompute_inertia/constraint_strength 를
  다 바꿔도) 를 씬에 넣으면 FEM.Elastic 이 다시 거의 완전히 얼어붙었다. Franka
  는 정상 동작해서 처음엔 "M0609 고유 문제"로 좁혀갔는데, **실제로는 FEM 바디의
  형상이 원인**이었다: tetgen 으로 사면체화된 메시(Sphere primitive 든, 정제
  STL 이든)는 M0609 앞에서 얼어붙지만, **`gs.morphs.Box` primitive 는 M0609
  앞에서도 정상 낙하**(격리 테스트로 실측: 30스텝만에 z 0.5→0.087, 완전한
  자유낙하 가속). 원인은 불명(Genesis 내부 tetgen 산출 메시 특유의 뭔가가
  M0609 의 IPC 커플링과 상호작용하는 것으로 추정, 후속 조사 필요)이지만
  실용적 해결책은 명확 — **정제를 Box primitive 로 근사**한다(사용자 지시).
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
TABLET_MODE = os.environ.get("TABLET_MODE", "box")  # "box" | "capsule_analytic"

ROBOT_MJCF = os.path.join(paths.ROBOTS_DIR, "m0609_rg2_v2.xml")
COACD_DIR_REL = "rg2/reference_onrobot_ros/meshes/rg2_v1/coacd"
FLEX_FINGER_HULLS = [f"flex_finger_hull_{i:03d}.stl" for i in range(7)]
BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag", "Samplebag_seal_pouch3.stl")

# 정제는 tetgen 산출 mesh(STL) 대신 Box primitive 로 근사(§docstring 2번째
# 발견 — M0609+FEM.Elastic 조합에서 tetgen mesh 만 얼어붙는 문제 회피).
# 실제 정제 치수(≈8x8x4mm)에 근접하되, 봉투 입구 간극(두께축 6mm)을 여유
# 있게 통과하도록 얇은 쪽을 6mm 이내로.
TABLET_SIZE = (0.006, 0.004, 0.006)

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"grasp_bag_tablet_ipc_{_TS}.mp4")

# ── 시뮬 옵션 ────────────────────────────────────────────────────────────────
DT = 5e-3
IPC_D_HAT = 1.0e-3
RENDER_EVERY = 4

# ── FEM cloth(봉투) — grasp_bag_ipc_test.py 검증값 ─────────────────────────
CLOTH_E, CLOTH_NU, CLOTH_RHO = 1.0e5, 0.499, 200.0
CLOTH_THICK, CLOTH_BEND = 1.0e-3, 50.0
CLOTH_FRICTION = 0.8

# ── FEM elastic(정제) — 공식 예제(ipc_robot_grasp_cube.py) 패턴, 파이프라인
# 검증 목적이라 무른 재질로 안정성 우선(§docstring 참고) ──────────────────
TABLET_E, TABLET_NU, TABLET_RHO = 5.0e4, 0.45, 1300.0
TABLET_FRICTION = 0.5

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
    """grasp_bag_ipc_test.py::_prepare_robot_mjcf 와 동일."""
    src_dir = paths.ROBOTS_DIR
    tmp_dir = tempfile.mkdtemp(prefix="m0609_ipc_bt_v2_")
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
    print(f" Tablet drop -> Bag catch -> M0609+RG2 grasp/lift (viewer={use_viewer})")
    print("=" * 60)

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        # fem_options 를 일부러 안 준다 — use_implicit_solver=True 로 켜면
        # Rigid 엔티티 존재 시 FEM 이 중력 적분을 멈추는 버그를 만난다
        # (§docstring, fem_tablet_drop.py 에서 격리 확인). 기본값(explicit)이
        # 공식 예제 ipc_robot_grasp_cube.py 와 동일하게 정상 동작한다.
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=IPC_D_HAT,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=False,
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
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.75,
                                     roughness=0.9, double_sided=True),
    )

    # 정제: 기본은 Box primitive(§docstring 2번째 발견). TABLET_MODE=capsule_analytic
    # 이면 TetGen을 아예 거치지 않는 해석적 캡슐(fan-tetrahedralization, §utills/
    # primitive_tablet_generator.py)로 대체해 "TetGen 산출물 자체가 원인인가"를
    # 직접 검증한다.
    tablet_material = gs.materials.FEM.Elastic(
        E=TABLET_E, nu=TABLET_NU, rho=TABLET_RHO,
        friction_mu=TABLET_FRICTION, model="stable_neohookean",
    )
    tablet_surface = gs.surfaces.Default(color=(0.9, 0.9, 0.85), roughness=0.6)
    if TABLET_MODE == "capsule_analytic":
        from primitive_tablet_generator import make_capsule_tets, add_analytic_fem_entity
        cap_verts_mm, cap_elems = make_capsule_tets(
            radius_mm=TABLET_SIZE[0] * 1e3 / 2, cyl_height_mm=TABLET_SIZE[2] * 1e3 * 0.3,
            n_theta=10, n_cap_rings=3,
        )
        print(f"[tablet] analytic capsule: verts={len(cap_verts_mm)} tets={len(cap_elems)} (TetGen 미사용)")
        tablet = add_analytic_fem_entity(
            scene, key=os.path.join(OUT_DIR, "_analytic_capsule.stl"),
            verts_mm=cap_verts_mm, elems=cap_elems,
            material=tablet_material, scale=1e-3, pos=TABLET_POS,
            surface=tablet_surface,
        )
    else:
        tablet = scene.add_entity(
            material=tablet_material,
            morph=gs.morphs.Box(pos=TABLET_POS, size=TABLET_SIZE),
            surface=tablet_surface,
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
            os.path.join(OUT_DIR, f"grasp_bag_tablet_ipc_{name}.png"))

    def _finger_z():
        return float(_npy(left_link.get_pos()).squeeze()[2])

    def _tablet_z():
        return float(_npy(tablet.get_state().pos).squeeze()[:, 2].mean())

    def run(name, q0, q1, f0, f1, n, trace=False):
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
        _shot(name)
        vpn = _npy(bag.get_state().pos).squeeze()
        gz = vpn[grip_idx, 2].mean() if len(grip_idx) else float("nan")
        print(f"[phase] {name:8s} @done  bag_com=({vpn[:,0].mean():.3f},{vpn[:,1].mean():.3f},"
              f"{vpn[:,2].mean():.3f})  grip_com_z={gz:.4f}  finger_z={_finger_z():.4f}  "
              f"tablet_z={_tablet_z()*1e3:+.2f}mm")
        return vpn

    # 1) drop: 그리퍼는 대기(open), 정제가 중력으로 낙하해 봉투 입구로 들어감
    run("drop", q_grasp, q_grasp, FING_OPEN, FING_OPEN, N_DROP, trace=True)
    # 2) settle: 정제가 봉투 안에서 안정화
    run("settle", q_grasp, q_grasp, FING_OPEN, FING_OPEN, N_SETTLE)
    # 3) close: 그리퍼가 실링부를 압착
    run("close", q_grasp, q_grasp, FING_OPEN, FING_CLOSE, N_CLOSE)
    vp_pre = run("grasp", q_grasp, q_grasp, FING_CLOSE, FING_CLOSE, N_GRASP)
    fz_pre = _finger_z()
    tz_pre = _tablet_z()
    # 4) lift: 봉투(+정제) 를 함께 들어올림
    run("lift", q_grasp, q_lift, FING_CLOSE, FING_CLOSE, N_LIFT)
    vp_post = run("hold", q_lift, q_lift, FING_CLOSE, FING_CLOSE, N_HOLD)
    fz_post = _finger_z()
    tz_post = _tablet_z()

    dz_f = fz_post - fz_pre
    dz_b = float(vp_post[grip_idx, 2].mean() - vp_pre[grip_idx, 2].mean()) if len(grip_idx) else float("nan")
    dz_t = tz_post - tz_pre
    print(f"\n[check] finger Δz={dz_f*1e3:+.1f}mm  bag grip Δz={dz_b*1e3:+.1f}mm  "
          f"tablet Δz={dz_t*1e3:+.1f}mm")
    bag_ok = len(grip_idx) > 0 and dz_f > 0.02 and dz_b > 0.5 * dz_f
    tablet_ok = dz_t > 0.5 * dz_f * 1e3 * 1e-3   # 정제도 봉투와 비슷하게 상승했는지
    print(f"[check] 봉투 파지: {'OK' if bag_ok else 'FAIL'}  /  정제 동반 상승: "
          f"{'OK' if tablet_ok else 'FAIL(봉투 밖으로 빠졌을 수 있음)'}")

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved video] {MP4}")
    print("[done] phase PNGs → M0609_RG2/RESULT/grasp_bag_tablet_ipc_<phase>.png")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

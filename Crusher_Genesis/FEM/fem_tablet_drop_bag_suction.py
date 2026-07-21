"""
fem_tablet_drop_bag_suction.py — fem_tablet_drop_bag_open.py 의 "입구 전체
밴드를 통째로 당기는" 방식이 부자연스럽다는 피드백에 따른 대안 버전.

배경(사용자 피드백)
------------------
넓은 밴드를 균일하게 당기면 탄성 범위를 벗어나는 변형(주름/처짐)이 안 보여서
부자연스럽다 — 실제로는 **석션(흡착) 컵 하나가 특정 지점 하나만 붙잡고
당기는** 방식이라, 그 한 점 주변에서 국소적인 주름/변형이 생기는 게 더
현실적이다. 이 버전은:
  1. 봉투 형태 유지용으로 바닥+양측면을 하드 고정(fem_tablet_drop_stiff.py
     계열과 달리 이번엔 계속 유지 — 서 있는 채로 두고 입구 변형만 관찰).
  2. 전면 패널 **상단 중앙 정점 1개**(SUCTION_N_VERTS로 주변 몇 개까지
     확장 가능)만 골라 -y(바깥) + 약간의 +z(들어올림) 방향으로
     PULL_DIST_SUCTION 만큼 당김 — 후면은 전혀 안 건드림(석션은 한쪽만 붙잡음).
  3. 입구가 국소적으로 벌어진 채로 정제(2배 스케일)를 자유낙하시켜본다.

fem_tablet_drop_bag_open.py 대비 바뀐 점: 밴드 전체 대신 단일(또는 소수)
정점만 당김, 당기는 거리도 더 작게(8mm), 바닥+측면은 계속 고정 유지.

출력: FEM/Result/fem_tablet_bag_suction_<ts>.mp4
"""
import os, sys
from datetime import datetime
import numpy as np

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

_DIR = os.path.dirname(os.path.abspath(__file__))
_r = _DIR
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
import paths

sys.path.insert(0, os.path.join(os.path.dirname(_r), "utills"))
from primitive_tablet_generator import make_capsule_tets_v2, add_analytic_fem_entity
from fem_ipc_workarounds import patch_fem_vertex_constraints

BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag", "Samplebag_seal_pouch3.stl")

DT = 5e-3
IPC_D_HAT = 1.0e-4

# ── 정제(2배 스케일, fem_tablet_drop_bag_open.py 동일) ──────────────────────
CAP_RADIUS_MM, CAP_CYL_H_MM = 4.0, 2.0
TABLET_E, TABLET_NU, TABLET_RHO = 5.0e4, 0.45, 1300.0
TABLET_FRICTION = 0.5

# ── 봉투(FEM.Cloth) ──────────────────────────────────────────────────────────
BAG_SCALE = 1.0
BAG_EULER = (90, 0, 0)      # local: X=폭(world X), Z=두께(world Y), Y=높이(world Z)
BAG_HALF_H = 0.045
BAG_POS = (0.0, 0.0, 0.06)

CLOTH_E, CLOTH_NU, CLOTH_RHO = 1.0e5, 0.499, 200.0
CLOTH_THICK, CLOTH_BEND = 1.0e-3, 50.0
CLOTH_FRICTION = 0.8

SHELF_TOP = BAG_POS[2] - BAG_HALF_H - 0.0015
SHELF_SIZE = (0.10, 0.10, 0.02)
SHELF_POS = (BAG_POS[0], BAG_POS[1], SHELF_TOP - SHELF_SIZE[2] / 2)

BAG_MOUTH_Z = BAG_POS[2] + BAG_HALF_H
TABLET_DROP_H = 0.020
TABLET_POS = (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z + TABLET_DROP_H)

# ── 형태 유지(바닥+양측면, full_workflow.py 방식) — 계속 유지 ───────────────
# 8mm 밴드로는 지지가 약해 정제 착지 충격에 봉투가 통째로 휙 쓰러졌다("점프"처럼
# 보인 원인) — 밴드를 넓혀 더 튼튼하게 서 있도록 함.
SHAPE_BAND = 0.020

# ── 석션 파라미터 ────────────────────────────────────────────────────────────
SUCTION_N_VERTS = 1           # 상단 중앙에서 가장 가까운 정점 몇 개를 붙잡을지(1=진짜 한 점)
PULL_DIST_Y = 0.008           # 바깥(-y)으로 8mm
LIFT_DIST_Z = 0.004           # 살짝 들어올림(+z) 4mm

N_OPEN = 100
N_HOLD_OPEN = 30
N_DROP = 200
N_SETTLE = 150

OUT_DIR = os.path.join(_DIR, "Result")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"fem_tablet_bag_suction_{_TS}.mp4")

CAM_POS, CAM_LOOK = (0.20, -0.20, 0.15), (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z - 0.01)
CAM_CLOSE_POS, CAM_CLOSE_LOOK = (0.06, -0.05, BAG_MOUTH_Z + 0.02), (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z - 0.005)


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" FEM Tablet(2x) drop with suction-cup single-point bag pull (viewer={use_viewer})")
    print("=" * 60)

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")
    patch_fem_vertex_constraints()

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=IPC_D_HAT,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=False,
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
        radius_mm=CAP_RADIUS_MM, cyl_height_mm=CAP_CYL_H_MM, n_theta=18, n_cap_rings=5, n_cyl_bands=2,
    )
    tablet = add_analytic_fem_entity(
        scene, key=os.path.join(OUT_DIR, "_analytic_capsule_2x_suction.stl"),
        verts_mm=cap_verts_mm, elems=cap_elems,
        material=gs.materials.FEM.Elastic(
            E=TABLET_E, nu=TABLET_NU, rho=TABLET_RHO,
            friction_mu=TABLET_FRICTION, model="stable_neohookean",
        ),
        scale=1e-3, pos=TABLET_POS,
        surface=gs.surfaces.Default(color=(0.9, 0.9, 0.85), roughness=0.6),
    )

    cam = scene.add_camera(res=(960, 720), pos=CAM_POS, lookat=CAM_LOOK, fov=38,
                            near=0.01, far=5.0, GUI=False)
    cam_close = scene.add_camera(res=(960, 720), pos=CAM_CLOSE_POS, lookat=CAM_CLOSE_LOOK, fov=28,
                                  near=0.005, far=5.0, GUI=False)

    print("\n[build] scene.build() 시작...")
    scene.build(n_envs=0)
    print("[build] 성공")

    bag_pos0 = _npy(bag.get_state().pos).squeeze()
    bx, by, bz = bag_pos0[:, 0], bag_pos0[:, 1], bag_pos0[:, 2]
    x_lo, x_hi = bx.min(), bx.max()
    y_lo, y_hi, z_hi = by.min(), by.max(), bz.max()
    x_mid = (x_lo + x_hi) / 2.0

    # ── 형태 유지(바닥+양측면), 입구 쪽은 그대로 자유 ───────────────────────
    bottom_mask = bz < bz.min() + SHAPE_BAND
    side_mask = (bx < x_lo + SHAPE_BAND) | (bx > x_hi - SHAPE_BAND)
    shape_idx = np.where(bottom_mask | side_mask)[0]
    bag.set_vertex_constraints(verts_idx_local=shape_idx.tolist(), is_soft_constraint=False)
    print(f"[bag] 형태 고정(바닥+양측면): {len(shape_idx)}/{len(bz)} 정점, 계속 유지")

    # ── 석션 지점: 전면(y=y_lo) 상단 중앙에 가장 가까운 정점 N_VERTS개 ──────
    target_pt = np.array([x_mid, y_lo, z_hi])
    front_face = by < (y_lo + y_hi) / 2.0
    dist = np.full(len(bag_pos0), np.inf)
    dist[front_face] = np.linalg.norm(bag_pos0[front_face] - target_pt, axis=1)
    suction_idx = np.argsort(dist)[:SUCTION_N_VERTS]
    print(f"[bag] 석션 지점: {len(suction_idx)}개 정점, pos={np.round(bag_pos0[suction_idx],4)}")

    # ── 정제도 같은 API로 공중에 붙잡아둔다 ─────────────────────────────────
    tab_pos0 = _npy(tablet.get_state().pos).squeeze()
    tab_idx_all = np.arange(len(tab_pos0))
    tablet.set_vertex_constraints(verts_idx_local=tab_idx_all.tolist(), target_poss=tab_pos0, is_soft_constraint=False)

    # 석션 제약은 shape 제약과 별개 호출로 등록(같은 정점이 겹치지 않게 미리 배제)
    suction_idx = np.array([i for i in suction_idx if i not in set(shape_idx.tolist())])
    if len(suction_idx) == 0:
        print("[bag][warn] 석션 후보가 전부 shape 고정 밴드와 겹침 — SHAPE_BAND를 줄이거나 위치 재검토 필요")
    else:
        suction_start = bag_pos0[suction_idx].copy()
        suction_target = suction_start.copy()
        suction_target[:, 1] -= PULL_DIST_Y
        suction_target[:, 2] += LIFT_DIST_Z
        bag.set_vertex_constraints(verts_idx_local=suction_idx.tolist(), target_poss=suction_start, is_soft_constraint=False)

    cam.start_recording()
    cam_close.start_recording()

    def render():
        cam.render()
        cam_close.render()

    def _bag_com():
        return _npy(bag.get_state().pos).squeeze().mean(axis=0)

    def _tab_com():
        return _npy(tablet.get_state().pos).squeeze().mean(axis=0)

    def _suction_pos():
        return _npy(bag.get_state().pos).squeeze()[suction_idx].mean(axis=0)

    # ── Phase 1: open — 석션 지점만 서서히 당김 ─────────────────────────────
    print(f"\n[phase] open ({N_OPEN}steps) — 석션 지점 {len(suction_idx)}개를 "
          f"-y {PULL_DIST_Y*1e3:.0f}mm / +z {LIFT_DIST_Z*1e3:.0f}mm")
    for k in range(N_OPEN):
        s = (k + 1) / N_OPEN
        if len(suction_idx) > 0:
            tgt = suction_start + (suction_target - suction_start) * s
            bag.update_constraint_targets(verts_idx_local=suction_idx.tolist(), target_poss=tgt)
        scene.step()
        render()
        if (k + 1) % 25 == 0:
            print(f"    k={k+1:4d} s={s:.2f} suction_pos={np.round(_suction_pos(),4)}")

    print(f"[phase] hold_open ({N_HOLD_OPEN}steps)")
    for k in range(N_HOLD_OPEN):
        scene.step()
        render()

    print(f"[phase] drop ({N_DROP}steps) — 정제 구속 해제, 자유낙하")
    tablet.remove_vertex_constraints()
    for k in range(N_DROP):
        scene.step()
        render()
        if (k + 1) % 40 == 0:
            print(f"    k={k+1:4d} tablet_com_z={_tab_com()[2]*1e3:+.2f}mm  bag_com={np.round(_bag_com(),4)}")

    print(f"[phase] settle ({N_SETTLE}steps) — 석션 지점만 해제(형태 고정은 유지)")
    if len(suction_idx) > 0:
        bag.remove_vertex_constraints(verts_idx_local=suction_idx.tolist())
    for k in range(N_SETTLE):
        scene.step()
        render()
        if (k + 1) % 40 == 0:
            print(f"    k={k+1:4d} tablet_com_z={_tab_com()[2]*1e3:+.2f}mm  bag_com={np.round(_bag_com(),4)}")

    tab_final = _tab_com()
    bag_final = _bag_com()
    print(f"\n[final] tablet_com={np.round(tab_final,4)}  bag_com={np.round(bag_final,4)}")
    print(f"[check] 정제가 봉투 입구 아래로 내려감: {tab_final[2] < BAG_MOUTH_Z}")

    cam.stop_recording(save_to_filename=MP4, fps=30)
    MP4_CLOSE = MP4.replace(".mp4", "_closeup.mp4")
    cam_close.stop_recording(save_to_filename=MP4_CLOSE, fps=30)
    print(f"\n[saved] {MP4}")
    print(f"[saved closeup] {MP4_CLOSE}")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

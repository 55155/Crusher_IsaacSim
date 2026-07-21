"""
fem_tablet_drop_bag_open.py — 정제 스케일 2배 + "손으로 봉투 입구를 벌려서
넣어주는" 듯한 움직임 테스트(FEM 전용, 로봇/Crusher 없음).

배경
----
PBD 솔버의 fix_particles_to_link(파티클을 특정 링크에 고정)에 대응하는 FEM
쪽 API가 FEMEntity.set_vertex_constraints(target_poss=..., is_soft_constraint=
False)다 — 특정 정점들을 원하는 목표 좌표로 하드 고정할 수 있다(§utills/
fem_ipc_workarounds.py, full_workflow.py 의 봉투 형상 고정과 동일 메커니즘).
이 스크립트는 그 API로:
  1. 봉투 입구(mouth) 근처의 전면/후면 정점을 골라 서로 반대 방향으로
     목표좌표를 매 스텝 보간(update_constraint_targets)하며 "벌어지는" 움직임을
     만들고,
  2. 그동안 정제도 같은 API로 공중에 붙잡아뒀다가, 입구가 다 벌어진 뒤에
     remove_vertex_constraints 로 놓아 자유낙하시키고,
  3. 정제가 안착하면 입구 고정도 풀어 봉투가 자연스럽게 다시 오므라들게 둔다.

정제는 CAP_RADIUS_MM/CAP_CYL_H_MM 을 full_workflow.py 대비 2배로 키웠다
(2.0/1.0mm -> 4.0/2.0mm, 지름 4mm×길이 5mm -> 지름 8mm×길이 10mm).

출력: FEM/Result/fem_tablet_bag_open_<ts>.mp4
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

# ── 정제(2배 스케일) ─────────────────────────────────────────────────────────
CAP_RADIUS_MM, CAP_CYL_H_MM = 4.0, 2.0     # full_workflow.py(2.0, 1.0) 의 2배
TABLET_E, TABLET_NU, TABLET_RHO = 5.0e4, 0.45, 1300.0
TABLET_FRICTION = 0.5

# ── 봉투(FEM.Cloth) — fem_tablet_drop_stiff.py 검증값 그대로 ────────────────
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
TABLET_DROP_H = 0.020        # 정제가 커진 만큼 여유도 살짝 키움
TABLET_POS = (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z + TABLET_DROP_H)

# ── 입구 "벌리기" 파라미터 ───────────────────────────────────────────────────
MOUTH_BAND = 0.012            # 입구에서 이 폭(m) 안쪽까지를 "벌릴 정점"으로 선택
PULL_DIST = 0.015             # 전/후면 각각 반대 방향으로 15mm씩 당김(총 갭 +30mm)

N_OPEN = 100                  # 입구가 벌어지는 동안(정제는 공중에 고정)
N_HOLD_OPEN = 30              # 벌어진 채로 잠깐 정지(형상 안정화)
N_DROP = 200                  # 정제 자유낙하 + 안착
N_SETTLE = 150                # 입구 고정 해제 후 봉투가 다시 오므라드는 구간

OUT_DIR = os.path.join(_DIR, "Result")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"fem_tablet_bag_open_{_TS}.mp4")

CAM_POS, CAM_LOOK = (0.20, -0.20, 0.15), (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z - 0.01)


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" FEM Tablet(2x scale) drop with bag-mouth-open test (viewer={use_viewer})")
    print("=" * 60)
    print(f"  capsule R={CAP_RADIUS_MM}mm cyl_h={CAP_CYL_H_MM}mm "
          f"(지름 {2*CAP_RADIUS_MM:.0f}mm x 길이 {CAP_CYL_H_MM+2*CAP_RADIUS_MM:.0f}mm)")

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
        scene, key=os.path.join(OUT_DIR, "_analytic_capsule_2x.stl"),
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

    print("\n[build] scene.build() 시작...")
    scene.build(n_envs=0)
    print("[build] 성공")

    # ── 봉투 입구 전/후면 정점 선택 (world: y=두께축, z=높이축) ────────────
    bag_pos0 = _npy(bag.get_state().pos).squeeze()
    by, bz = bag_pos0[:, 1], bag_pos0[:, 2]
    y_lo, y_hi, z_hi = by.min(), by.max(), bz.max()
    front_mask = (by < y_lo + 0.0015) & (bz > z_hi - MOUTH_BAND)
    back_mask = (by > y_hi - 0.0015) & (bz > z_hi - MOUTH_BAND)
    front_idx = np.where(front_mask)[0]
    back_idx = np.where(back_mask)[0]
    open_idx = np.concatenate([front_idx, back_idx])
    print(f"[bag] 입구 벌릴 정점: front={len(front_idx)} back={len(back_idx)} "
          f"(mouth_band={MOUTH_BAND*1e3:.0f}mm, 봉투 두께 y=[{y_lo*1e3:.2f},{y_hi*1e3:.2f}]mm)")

    open_start = bag_pos0[open_idx].copy()
    open_target = open_start.copy()
    open_target[:len(front_idx), 1] -= PULL_DIST     # front -> -y
    open_target[len(front_idx):, 1] += PULL_DIST     # back  -> +y

    # ── 정제도 같은 API로 공중에 붙잡아둔다(벌어지는 동안 미리 낙하 방지) ──
    tab_pos0 = _npy(tablet.get_state().pos).squeeze()
    tab_idx_all = np.arange(len(tab_pos0))

    bag.set_vertex_constraints(verts_idx_local=open_idx.tolist(), target_poss=open_start, is_soft_constraint=False)
    tablet.set_vertex_constraints(verts_idx_local=tab_idx_all.tolist(), target_poss=tab_pos0, is_soft_constraint=False)

    cam.start_recording()

    def render():
        cam.render()

    def _bag_com():
        return _npy(bag.get_state().pos).squeeze().mean(axis=0)

    def _tab_com():
        return _npy(tablet.get_state().pos).squeeze().mean(axis=0)

    # ── Phase 1: open — 입구를 서서히 벌림(정제는 고정 유지) ───────────────
    print(f"\n[phase] open ({N_OPEN}steps) — 입구 전/후면을 서로 반대 방향으로 {PULL_DIST*1e3:.0f}mm씩")
    for k in range(N_OPEN):
        s = (k + 1) / N_OPEN
        tgt = open_start + (open_target - open_start) * s
        bag.update_constraint_targets(verts_idx_local=open_idx.tolist(), target_poss=tgt)
        scene.step()
        render()
        if (k + 1) % 25 == 0:
            print(f"    k={k+1:4d} s={s:.2f} bag_com={np.round(_bag_com(),4)}")

    # ── Phase 2: hold open — 벌어진 채로 잠깐 정지 ──────────────────────────
    print(f"[phase] hold_open ({N_HOLD_OPEN}steps)")
    for k in range(N_HOLD_OPEN):
        scene.step()
        render()

    # ── Phase 3: 정제 놓아주기 — 자유낙하 ───────────────────────────────────
    print(f"[phase] drop ({N_DROP}steps) — 정제 구속 해제, 자유낙하")
    tablet.remove_vertex_constraints()
    for k in range(N_DROP):
        scene.step()
        render()
        if (k + 1) % 40 == 0:
            print(f"    k={k+1:4d} tablet_com_z={_tab_com()[2]*1e3:+.2f}mm  bag_com={np.round(_bag_com(),4)}")

    # ── Phase 4: 입구 고정 해제 — 봉투가 자연스럽게 다시 오므라듦 ───────────
    print(f"[phase] settle ({N_SETTLE}steps) — 입구 구속 해제, 봉투 자연 이완")
    bag.remove_vertex_constraints()
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
    print(f"\n[saved] {MP4}")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

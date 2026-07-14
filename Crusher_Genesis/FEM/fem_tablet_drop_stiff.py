"""
fem_tablet_drop_stiff.py — fem_tablet_drop.py(sliver-free v2 캡슐, 3fca8c8)의
"안정성 우선으로 낮춘 E=5e4" 를 벗어나, **FEM(정제) + FEM(샘플백) + IPC**
조합을 유지한 채 강성을 실제 정제에 더 가깝게 올려서 재도전한다.

**배경**: rigid_capsule_tablet_bag_ipc_test.py 로 Rigid+SDF capsule 조합의
발산을 `coup_type="ipc_only"` 로 우회했지만(§DigitalTwin.md §8), 그건 캡슐이
"변형되지 않는 강체"라는 뜻이라 실제 정제(변형체)의 압축·파손 거동은 못 본다.
FEM(변형체) 캡슐로 남되 강성만 올리는 이 스크립트가 "정제다운 정제"에 더
가깝다.

**강성 선택 근거**: explicit FEM 적분의 안정 dt는 대략 파동속도
c=sqrt(E/ρ)에 반비례(dt ∝ 1/sqrt(E)) 하므로, 기존 안정 조합(E=5e4, dt=5e-3)
을 기준으로 `dt_new = dt_base * sqrt(E_base / E_new)` 로 스케일링하고,
물리 시간(≈1.5s)을 유지하도록 스텝 수도 같이 늘린다. 문헌값 실제 정제
E~2GPa(§DigitalTwin.md §7)는 이 스케일링으로도 dt가 25μs까지 줄어 스텝 수가
비현실적으로 커지므로(§grasp_bag_tablet_ipc_test.py 에서도 발산 보고),
이 스크립트는 **E를 계단식으로 올려가며 어디서 깨지는지 찾는 용도**다.

사용법: `TABLET_E=1e6 python fem_tablet_drop_stiff.py` (기본 1e6 = 5e4 대비 20배)

출력: FEM/Result/fem_tablet_bag_drop_stiff_E<E>_<ts>.mp4
       + 매 스텝 로그로 발산(좌표 폭주) 여부 추적.
"""
import os, sys, gc
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
from primitive_tablet_generator import make_capsule_tets, make_capsule_tets_v2, add_analytic_fem_entity

# CAPSULE_VARIANT=v1 -> 순수 fan tetrahedralization(전역 centroid, 알려진 sliver 있음)
# CAPSULE_VARIANT=v2(기본) -> medial-axis 다중 앵커(sliver-free)
CAPSULE_VARIANT = os.environ.get("CAPSULE_VARIANT", "v2")

BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag", "Samplebag_seal_pouch3.stl")

# ── 강성 스케일링 ────────────────────────────────────────────────────────────
E_BASE, DT_BASE = 5.0e4, 5e-3       # fem_tablet_drop.py 검증된 안정 조합
SIM_DURATION = float(os.environ.get("SIM_DURATION", "1.5"))  # s
TARGET_FRAMES = 150                  # 영상 프레임 수(대략) — 렌더러 MemoryError 회피 위해 축소

TABLET_E = float(os.environ.get("TABLET_E", "1e6"))
TABLET_NU, TABLET_RHO = 0.45, 1300.0
TABLET_FRICTION = 0.5

DT = DT_BASE * (E_BASE / TABLET_E) ** 0.5
N_STEPS = max(int(round(SIM_DURATION / DT)), 10)
N_DROP = int(N_STEPS * 0.5)
N_SETTLE = N_STEPS - N_DROP
RENDER_EVERY = max(int(round(N_STEPS / TARGET_FRAMES)), 1)

CAP_RADIUS_MM, CAP_CYL_H_MM = 2.0, 1.0

# ── 봉투(FEM.Cloth) — fem_tablet_drop.py 검증값 ─────────────────────────
BAG_SCALE = 1.0
BAG_EULER = (90, 0, 0)
BAG_HALF_H = 0.045
BAG_POS = (0.0, 0.0, 0.06)

SHELF_TOP = BAG_POS[2] - BAG_HALF_H - 0.0015
SHELF_SIZE = (0.10, 0.10, 0.02)
SHELF_POS = (BAG_POS[0], BAG_POS[1], SHELF_TOP - SHELF_SIZE[2] / 2)

CLOTH_E, CLOTH_NU, CLOTH_RHO = 1.0e5, 0.499, 200.0
CLOTH_THICK, CLOTH_BEND = 1.0e-3, 50.0
CLOTH_FRICTION = 0.8

BAG_MOUTH_Z = BAG_POS[2] + BAG_HALF_H
TABLET_DROP_H = 0.015
TABLET_POS = (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z + TABLET_DROP_H)

OUT_DIR = os.path.join(_DIR, "Result")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"fem_tablet_bag_drop_stiff_{CAPSULE_VARIANT}_E{TABLET_E:.0e}_{_TS}.mp4")

CAM_POS, CAM_LOOK = (0.16, -0.16, 0.13), (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z - 0.01)


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" FEM Tablet (STIFF, {CAPSULE_VARIANT}, E={TABLET_E:.2e}) -> Samplebag Drop Test (viewer={use_viewer})")
    print("=" * 60)
    print(f"  E_base={E_BASE:.1e} -> E={TABLET_E:.2e}  ({TABLET_E/E_BASE:.0f}x)")
    print(f"  dt scaled {DT_BASE*1e3:.3f}ms -> {DT*1e3:.4f}ms   N_STEPS={N_STEPS}  RENDER_EVERY={RENDER_EVERY}")
    print(f"  capsule R={CAP_RADIUS_MM}mm cyl_h={CAP_CYL_H_MM}mm")

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=float(os.environ.get("D_HAT", "5.0e-4")),
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
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.4,
                                     roughness=0.9, double_sided=True),
    )

    if CAPSULE_VARIANT == "v1":
        cap_verts_mm, cap_elems = make_capsule_tets(
            radius_mm=CAP_RADIUS_MM, cyl_height_mm=CAP_CYL_H_MM, n_theta=12, n_cap_rings=4,
        )
        print(f"[tablet]  capsule verts={len(cap_verts_mm)} tets={len(cap_elems)} (v1 fan tetrahedralization, sliver 있음)")
    else:
        cap_verts_mm, cap_elems = make_capsule_tets_v2(
            radius_mm=CAP_RADIUS_MM, cyl_height_mm=CAP_CYL_H_MM, n_theta=12, n_cap_rings=4, n_cyl_bands=2,
        )
        print(f"[tablet]  capsule verts={len(cap_verts_mm)} tets={len(cap_elems)} (v2 sliver-free medial-axis)")

    tablet = add_analytic_fem_entity(
        scene, key=os.path.join(OUT_DIR, f"_analytic_capsule_{CAPSULE_VARIANT}_stiff.stl"),
        verts_mm=cap_verts_mm, elems=cap_elems,
        material=gs.materials.FEM.Elastic(
            E=TABLET_E, nu=TABLET_NU, rho=TABLET_RHO,
            friction_mu=TABLET_FRICTION, model="stable_neohookean",
        ),
        scale=1e-3, pos=TABLET_POS,
        surface=gs.surfaces.Default(color=(0.9, 0.9, 0.85), roughness=0.6),
    )

    cam = scene.add_camera(res=(640, 480), pos=CAM_POS, lookat=CAM_LOOK, fov=35,
                            near=0.01, far=5.0, GUI=False)

    # CLOSEUP=1: 정제를 따라다니는 초근접 카메라 — "우그러짐"이 실제 형상
    # 왜곡인지 아니면 저해상도 렌더링(작은 화면 점유율) 착시인지 진단용.
    CLOSEUP = os.environ.get("CLOSEUP") == "1"
    cam_close = None
    if CLOSEUP:
        cam_close = scene.add_camera(res=(800, 800), pos=TABLET_POS, lookat=TABLET_POS,
                                      fov=8, near=0.001, far=1.0, GUI=False)

    scene.build(n_envs=0)

    vp0 = _npy(bag.get_state().pos).squeeze()
    print(f"[bag]     z={vp0[:,2].min():.4f}~{vp0[:,2].max():.4f}")
    pos0 = _npy(tablet.get_state().pos).squeeze()
    x0span = (pos0[:, 0].max() - pos0[:, 0].min()) * 1e3
    y0span = (pos0[:, 1].max() - pos0[:, 1].min()) * 1e3
    z0span = (pos0[:, 2].max() - pos0[:, 2].min()) * 1e3
    print(f"[tablet]  nodes={len(pos0)}  z0_mean={pos0[:,2].mean()*1e3:.2f}mm  "
          f"rest bbox(mm) x={x0span:.3f} y={y0span:.3f} z={z0span:.3f}  "
          f"(설계값: xy={2*CAP_RADIUS_MM:.1f} z={CAP_CYL_H_MM+2*CAP_RADIUS_MM:.1f})")

    DUMP_VERTS = os.environ.get("DUMP_VERTS") == "1"
    vert_dump = {}
    dump_at_ms = (0, 75, 300, 750, 1499)
    _dumped = set()

    cam.start_recording()
    if cam_close is not None:
        cam_close.start_recording()
    diverged = False
    render_ok = True
    log_every = max(N_STEPS // 20, 1)
    if DUMP_VERTS:
        vert_dump["t000_rest"] = pos0.copy()
    for k in range(N_STEPS):
        scene.step()
        if render_ok and (k + 1) % RENDER_EVERY == 0:
            try:
                cam.render()
                if cam_close is not None:
                    p_now = _npy(tablet.get_state().pos).squeeze().mean(axis=0)
                    look = tuple(p_now)
                    eye = (p_now[0] + 0.012, p_now[1] - 0.012, p_now[2] + 0.006)
                    cam_close.set_pose(pos=eye, lookat=look)
                    cam_close.render()
            except Exception as e:
                render_ok = False
                print(f"  [!] 렌더링 실패(물리와 무관, 이후 렌더 스킵): {e}")
            if (k + 1) % (RENDER_EVERY * 20) == 0:
                gc.collect()
        if DUMP_VERTS:
            t_ms = (k + 1) * DT * 1e3
            for target in dump_at_ms:
                if target not in _dumped and t_ms >= target:
                    _dumped.add(target)
                    vert_dump[f"t{target:04d}"] = _npy(tablet.get_state().pos).squeeze().copy()
        if (k + 1) % log_every == 0:
            p = _npy(tablet.get_state().pos).squeeze()
            xc, xmin, xmax = p[:, 0].mean(), p[:, 0].min(), p[:, 0].max()
            yc, ymin, ymax = p[:, 1].mean(), p[:, 1].min(), p[:, 1].max()
            zc, zmin, zmax = p[:, 2].mean(), p[:, 2].min(), p[:, 2].max()
            xspan, yspan, zspan = (xmax - xmin) * 1e3, (ymax - ymin) * 1e3, (zmax - zmin) * 1e3
            phase = "drop" if k < N_DROP else "settle"
            print(f"  [{phase}] t={(k+1)*DT*1e3:6.1f}ms  tablet_com_z={zc*1e3:+.3f}mm  "
                  f"bbox(mm) x={xspan:.3f} y={yspan:.3f} z={zspan:.3f}  min_z={zmin*1e3:+.3f}mm")
            if not np.isfinite(p).all() or np.abs(p).max() > 1.0:
                diverged = True
                print(f"  [!!] 발산 감지 — 조기 종료")
                break

    pf = _npy(tablet.get_state().pos).squeeze()
    bagf = _npy(bag.get_state().pos).squeeze()
    fall_mm = (pos0[:, 2].mean() - pf[:, 2].mean()) * 1e3
    inside_xy = (pf[:, 0].mean() - BAG_POS[0]) ** 2 + (pf[:, 1].mean() - BAG_POS[1]) ** 2 < 0.03 ** 2
    at_or_below_mouth = pf[:, 2].mean() <= BAG_MOUTH_Z + 0.002
    print(f"\n[final] E={TABLET_E:.2e}  diverged={diverged}  tablet_com_z={pf[:,2].mean()*1e3:.3f}mm  "
          f"net_fall={fall_mm:.3f}mm  bag_com_z={bagf[:,2].mean()*1e3:.3f}mm")
    print(f"[check] 봉투 입구 높이 이하: {at_or_below_mouth}  /  봉투 중심 근방(xy): {inside_xy}")

    if render_ok:
        cam.stop_recording(save_to_filename=MP4, fps=30)
        print(f"\n[saved video] {MP4}")
        if cam_close is not None:
            MP4_CLOSE = MP4.replace(".mp4", "_closeup.mp4")
            cam_close.stop_recording(save_to_filename=MP4_CLOSE, fps=30)
            print(f"[saved closeup video] {MP4_CLOSE}")
    else:
        print("\n[skip video] 렌더링이 도중 실패해 저장 안 함(물리 로그는 위 [drop]/[settle] 참고)")

    if DUMP_VERTS:
        npz_path = os.path.join(OUT_DIR, f"_verts_dump_{CAPSULE_VARIANT}_E{TABLET_E:.0e}_{_TS}.npz")
        np.savez(npz_path, elems=np.asarray(cap_elems), **vert_dump)
        print(f"[saved vert dump] {npz_path}  keys={list(vert_dump.keys())}")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

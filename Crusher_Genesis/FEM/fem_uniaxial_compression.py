"""
fem_uniaxial_compression.py — 정제 단축 압축 (공식 예제 패턴: soft vertex constraint)

공식 예제 `fem_hard_and_soft_constraint.py` 의 표준 패턴:
  - FEMOptions(use_implicit_solver=True, enable_vertex_constraints=True)
  - tablet.set_vertex_constraints(idx, target_positions, is_soft_constraint=True, stiffness=K)
  - 매 step tablet.update_constraint_targets(idx, new_target) 로 BC 갱신

자유도:
  - 정제 top 노드: 강성 K 스프링으로 target 위치에 끌림 (시간에 따라 -z 이동)
  - 정제 bot 노드: 강성 K 스프링으로 초기 위치 고정 (Dirichlet)
  - 정제 내부: 모두 자유 (FEM)
  - Plate: 시각 보조 (target 위치 따라 그림자처럼 이동)

측정:
  - d(t)        : target 변위 [m]
  - top_z, bot_z: 정제 표면 노드 평균 z [m]
  - ΔH(t)       : 정제 두께 변화 [m]
  - ε(t)        : 공학 strain [-]
  - W_int(t)    : FEM strain energy [J] (solver field)

출력:
  Sim_result/fem_uniaxial_<ts>.mp4
  Sim_result/fem_uniaxial_<ts>.png
"""
import os, sys
from datetime import datetime
import numpy as np
import trimesh as tm
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Windows cp949
for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

# ── 옵션 (테스트 단계 — mesh 거칠게 + 짧은 시뮬) ───────────────
DT, SUBSTEPS = 1e-3, 1            # implicit: 큰 dt OK
PLATE_VEL    = 2.0e-3             # 2 mm/s (더 빨라야 짧은 시뮬에서 변형 보임)
DURATION     = 0.3                # 0.3 s → 0.6 mm 변위
N_STEPS      = int(DURATION / DT)
RENDER_EVERY = 1
SAMPLE_EVERY = 3
TARGET_FACES = 200                # STL decimation: 원본 ~수천 face → 200 face
                                  # → tet 수 ~12000 → ~수백 으로 감소 예상

# 재료 (literature 정제 일반값)
# Soft constraint 의 stiffness K 는 정제 강성과 비슷해야 BC 가 효과 있음.
# K_tablet = E·A/L ≈ 2e9 · 5e-5 / 2.5e-3 = 4e7 N/m. K_bc 는 이 정도 또는 조금 강하게.
E_TABLET   = 2.0e9    # 2 GPa
NU_TABLET  = 0.25
RHO_TABLET = 1300.0   # kg/m^3
BC_STIFF   = 1.0e7    # N/m  (soft constraint 강성)

# 정제 STL
TABLET_STL   = "C:/Crusher_isaacsim/tablets_stl/stl/tablet_R4.0_AR1.00_CV0.20.stl"
TABLET_SCALE = 1e-3   # mm → m

# Plate (시각 + 압축 driver)
PLATE_SIZE   = (0.025, 0.025, 0.003)
PLATE_MARGIN = 0.001    # 정제 bbox 위·아래로 추가 여유 (초기 관통 0 보장)

# 경로
_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "..", "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"fem_uniaxial_{_TS}.mp4")
PNG = os.path.join(OUT_DIR, f"fem_uniaxial_{_TS}.png")


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def main(use_viewer: bool = False):
    print("="*60)
    print(f" FEM Uniaxial Compression  (viewer={use_viewer})")
    print("="*60)
    print(f"  E={E_TABLET/1e9:.1f} GPa  ν={NU_TABLET}  ρ={RHO_TABLET} kg/m³")
    print(f"  v_plate={PLATE_VEL*1000:.2f} mm/s  duration={DURATION:.2f} s")
    print(f"  STL = {os.path.basename(TABLET_STL)}")

    # ── STL load + decimate (테스트 단계: 노드 대폭 축소) ─────────
    raw_mesh = tm.load(TABLET_STL)
    n_face_orig = len(raw_mesh.faces)
    if n_face_orig > TARGET_FACES:
        try:
            raw_mesh = raw_mesh.simplify_quadric_decimation(face_count=TARGET_FACES)
        except Exception as e:
            print(f"[decimate][warn] simplify failed: {e} — using original")
    n_face_new = len(raw_mesh.faces)
    # 임시 STL 저장 (Genesis 가 file path 요구)
    STL_TMP = os.path.join(OUT_DIR, f"_tablet_decimated_{TARGET_FACES}.stl")
    raw_mesh.export(STL_TMP)
    print(f"[stl] decimate {n_face_orig} → {n_face_new} faces  → {STL_TMP}")

    bb = raw_mesh.bounding_box.bounds * TABLET_SCALE   # (2, 3), m 단위
    tab_lo, tab_hi = bb[0], bb[1]
    tab_h = float(tab_hi[2] - tab_lo[2])
    print(f"[stl] bbox  x=[{tab_lo[0]*1e3:.2f},{tab_hi[0]*1e3:.2f}]  "
          f"y=[{tab_lo[1]*1e3:.2f},{tab_hi[1]*1e3:.2f}]  z=[{tab_lo[2]*1e3:.2f},{tab_hi[2]*1e3:.2f}] mm")
    print(f"[stl] z thickness = {tab_h*1e3:.2f} mm")
    # 정제 중심을 (0,0,tab_center_z) 로 두기 위한 placement offset
    tablet_pos = (-0.5 * (tab_lo[0] + tab_hi[0]),
                  -0.5 * (tab_lo[1] + tab_hi[1]),
                  -0.5 * (tab_lo[2] + tab_hi[2]))      # bbox center 를 원점에 정렬
    # 동적 plate gap = 정제 두께 + 양쪽 margin
    plate_gap = tab_h + 2 * PLATE_MARGIN
    print(f"[plate] gap = {plate_gap*1e3:.2f} mm  (margin {PLATE_MARGIN*1e3:.2f} mm each side)")

    import genesis as gs
    _backend = gs.metal if sys.platform == "darwin" else gs.cuda
    gs.init(backend=_backend, precision="64", logging_level="warning")  # SAP requires 64bit

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, -9.81)),
        fem_options=gs.options.FEMOptions(
            use_implicit_solver=True,         # backward Euler
            enable_vertex_constraints=True,   # *** 필수 *** (디폴트 False)
        ),
        coupler_options=gs.options.SAPCouplerOptions(),   # Drake SAP contact
        vis_options=gs.options.VisOptions(background_color=(0.95, 0.95, 0.97)),
        show_viewer=use_viewer,
    )

    # 정제 중심을 (0, 0, plate_gap/2) 에 위치 (gap 정중앙)
    z_center = plate_gap / 2
    # 하부 plate (정적, 정제 -z 면에서 PLATE_MARGIN 만큼 떨어짐)
    plate_bot_z = -PLATE_MARGIN - PLATE_SIZE[2] / 2
    plate_bot = scene.add_entity(
        morph=gs.morphs.Box(size=PLATE_SIZE, pos=(0, 0, plate_bot_z), fixed=True),
        material=gs.materials.Rigid(),
        surface=gs.surfaces.Default(color=(0.45, 0.45, 0.55)),
    )
    # 상부 plate (movable: IPC 의 ABD 가 ≥ 1 movable body 요구)
    plate_top_z0 = plate_gap + PLATE_MARGIN + PLATE_SIZE[2] / 2
    plate_top = scene.add_entity(
        morph=gs.morphs.Box(size=PLATE_SIZE, pos=(0, 0, plate_top_z0), fixed=False),
        material=gs.materials.Rigid(),
        surface=gs.surfaces.Default(color=(0.70, 0.45, 0.45)),
    )
    # 정제 (FEM) — bbox center 를 (0, 0, z_center) 에 정확히 배치
    tablet_morph_pos = (tablet_pos[0], tablet_pos[1], tablet_pos[2] + z_center)
    tablet = scene.add_entity(
        material=gs.materials.FEM.Elastic(
            E=E_TABLET, nu=NU_TABLET, rho=RHO_TABLET, model="linear_corotated",
        ),
        morph=gs.morphs.Mesh(
            file=STL_TMP, scale=TABLET_SCALE,
            pos=tablet_morph_pos,
        ),
    )

    # 카메라: 정제와 같은 높이에서 측면 촬영 (contact·압축 보기 좋음)
    # scene 이 mm 단위라 카메라는 ~20cm 거리 + 좁은 fov 6° → frame 의 ~40% 가 entity 영역
    z_eye = plate_gap / 2                       # 정제 중심 높이
    scene_center = np.array([0.0, 0.0, z_eye])
    cam_pos = (0.02, -0.20, z_eye)              # x 살짝 (2cm) 비대칭으로 깊이감
    cam_fov = 6
    print(f"[cam]  side-view  pos={cam_pos}  lookat={tuple(scene_center)}  fov={cam_fov}")
    cam = scene.add_camera(
        res=(960, 720),
        pos=cam_pos,
        lookat=tuple(scene_center),
        fov=cam_fov, GUI=False,
    )

    scene.build(n_envs=0)

    # 빌드 후 노드 위치 (변형 추적 + BC 그룹 식별)
    import torch
    pos0 = _npy(tablet.get_state().pos).squeeze()  # (N, 3)
    z = pos0[:, 2]
    z_lo, z_hi = z.min(), z.max()
    band = (z_hi - z_lo) * 0.10
    top_idx = np.where(z > z_hi - band)[0].astype(int).tolist()
    bot_idx = np.where(z < z_lo + band)[0].astype(int).tolist()
    print(f"\n[tablet] nodes={len(pos0)}  z=[{z_lo*1000:.2f}, {z_hi*1000:.2f}] mm")
    print(f"[bc]     top nodes={len(top_idx)}, bot nodes={len(bot_idx)}  (band={band*1e3:.2f} mm)")

    # ── 공식 예제 패턴: soft vertex constraint ────────────────────
    # init_positions 가 torch tensor → 그 위에 offset 더해서 target 만듦
    init_pos = tablet.init_positions   # torch tensor (N, 3)
    dev, dt_ = init_pos.device, init_pos.dtype

    bot_init = init_pos[bot_idx].clone()
    top_init = init_pos[top_idx].clone()

    # 하면 노드: 초기 위치 유지 (Dirichlet)
    tablet.set_vertex_constraints(
        bot_idx, bot_init,
        is_soft_constraint=True, stiffness=BC_STIFF,
    )
    # 상면 노드: 초기 위치에서 시작, 매 step -z 로 갱신
    tablet.set_vertex_constraints(
        top_idx, top_init,
        is_soft_constraint=True, stiffness=BC_STIFF,
    )
    print(f"[bc]     soft constraint stiffness K = {BC_STIFF:.1e} N/m")

    # FEM solver 접근 (energy field probe)
    fem_solver = scene.sim.fem_solver

    # 데이터 기록
    t_arr, d_arr, top_z_arr, bot_z_arr, W_int_arr = [], [], [], [], []

    cam.start_recording()
    print("\n[run] stepping...")
    for step in range(N_STEPS):
        t = step * DT
        d = PLATE_VEL * t
        new_z = plate_top_z0 - d

        # 1) 상면 노드 target 갱신 (-z 로 d 만큼)
        offset = torch.tensor([0.0, 0.0, -d], device=dev, dtype=dt_)
        new_top_target = top_init + offset
        tablet.update_constraint_targets(top_idx, new_top_target)

        # 2) Plate 시각 동기화 (set_pos: fixed=True 라도 텔레포트 OK)
        plate_top.set_pos(np.array([0.0, 0.0, new_z]), zero_velocity=True)

        scene.step()

        if step % SAMPLE_EVERY == 0:
            pos = _npy(tablet.get_state().pos).squeeze()
            top_z_now = float(pos[top_idx, 2].mean())
            bot_z_now = float(pos[bot_idx, 2].mean())
            try:
                e_dict = fem_solver.elements_el_energy.to_numpy()
                W_int = float(np.asarray(e_dict["energy"]).sum())
            except Exception:
                W_int = float("nan")

            t_arr.append(t)
            d_arr.append(d * 1e3)
            top_z_arr.append(top_z_now * 1e3)
            bot_z_arr.append(bot_z_now * 1e3)
            W_int_arr.append(W_int * 1e3)

            if step % (SAMPLE_EVERY * 50) == 0:
                tab_h = (top_z_now - bot_z_now) * 1e3
                print(f"  t={t*1e3:6.1f} ms  d={d*1e3:.3f} mm  top_z={top_z_now*1e3:.3f}mm  "
                      f"tablet_h={tab_h:.3f}mm  W_int={W_int*1e3:.3f} mJ")

        if step % RENDER_EVERY == 0:
            cam.render()

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved video] {MP4}")

    # ── Plot ───────────────────────────────────────────────────
    t_arr = np.array(t_arr); d_arr = np.array(d_arr)
    top_z_arr = np.array(top_z_arr); bot_z_arr = np.array(bot_z_arr)
    W_int_arr = np.array(W_int_arr)

    tablet_h = top_z_arr - bot_z_arr     # 정제 두께
    H0 = tablet_h[0] if len(tablet_h) else 1
    eps = (H0 - tablet_h) / H0           # nominal strain

    fig, ax = plt.subplots(2, 2, figsize=(12, 8))

    ax[0,0].plot(t_arr*1e3, d_arr, 'r-', lw=1.5, label="$d$ (plate)")
    ax[0,0].plot(t_arr*1e3, H0 - tablet_h, 'b--', lw=1.5, label="$\\Delta H$ (tablet compression)")
    ax[0,0].set_xlabel("Time [ms]"); ax[0,0].set_ylabel("Displacement [mm]")
    ax[0,0].set_title("(a) Plate vs tablet compression")
    ax[0,0].legend(); ax[0,0].grid(alpha=0.3)

    ax[0,1].plot(t_arr*1e3, eps*100, 'g-', lw=1.5)
    ax[0,1].set_xlabel("Time [ms]"); ax[0,1].set_ylabel("Engineering strain $\\varepsilon$ [%]")
    ax[0,1].set_title("(b) Nominal compressive strain")
    ax[0,1].grid(alpha=0.3)

    ax[1,0].plot(t_arr*1e3, top_z_arr, 'r-', lw=1.5, label="top nodes")
    ax[1,0].plot(t_arr*1e3, bot_z_arr, 'b-', lw=1.5, label="bot nodes")
    ax[1,0].set_xlabel("Time [ms]"); ax[1,0].set_ylabel("z [mm]")
    ax[1,0].set_title("(c) Tablet surface node tracking")
    ax[1,0].legend(); ax[1,0].grid(alpha=0.3)

    ax[1,1].plot(t_arr*1e3, W_int_arr, 'k-', lw=1.5)
    ax[1,1].set_xlabel("Time [ms]"); ax[1,1].set_ylabel("$W_{int}$ [mJ]")
    ax[1,1].set_title("(d) FEM strain energy (solver field)")
    ax[1,1].grid(alpha=0.3)

    fig.suptitle(f"FEM Uniaxial (SAP coupler) — {os.path.basename(TABLET_STL)}\n"
                 f"$E$={E_TABLET/1e9:.1f}GPa  $\\nu$={NU_TABLET}  $v$={PLATE_VEL*1e3:.2f}mm/s")
    plt.tight_layout()
    plt.savefig(PNG, dpi=120, bbox_inches='tight')
    plt.close()
    print(f"[saved plot]  {PNG}")

    print(f"\n[summary] d_max={d_arr.max():.3f}mm  ΔH_max={(H0-tablet_h.min()):.3f}mm  "
          f"ε_max={eps.max()*100:.2f}%  W_int_max={np.nanmax(W_int_arr):.3f}mJ")


if __name__ == "__main__":
    main(use_viewer=False)

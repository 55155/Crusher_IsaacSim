"""
fem_uniaxial_compression.py — 정제 단축 압축 (SAP coupler 실제 Rigid–FEM 접촉)

공식 예제 `sap_coupling/franka_grasp_fem_sphere.py` 패턴:
  - coupler_options=SAPCouplerOptions(...) — *진짜* 접촉 계산
  - Rigid(coup_friction=..., friction=...)  + FEM.Elastic(friction_mu=..., ...)
  - top 노드: vertex constraint 없음 → 평판과 SAP contact 로 거동
  - bot 노드: vertex constraint(link=plate_bot) → 하드 Dirichlet (경계조건)

자유도:
  - 정제 top: 자유 (FEM) — 평판과 SAP 접촉으로 압축됨
  - 정제 bot: hard Dirichlet (plate_bot 추종, 정적)
  - Plate top: 운동학적 텔레포트(set_pos)로 일정 속도 하강 → SAP 가 정제 표면 노드 푸시

측정:
  - d(t)        : 평판 변위 [m]
  - top_z, bot_z: 정제 표면 노드 평균 z [m]
  - ΔH(t)       : 정제 두께 변화 [m]
  - ε(t)        : 공학 strain [-]
  - σ(t)        : nominal stress = E·ε [MPa]
  - F(t)        : 평판→정제 수직 반력 [N]
                   = -Σ_top  coupler.fem_state_v.impulse[i,2] / dt
  - W(t)        : 평판이 정제에 가한 누적 일 [mJ]
                   = ∫ |F(t)| · v_plate dt   (사다리꼴)

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

# ── 옵션 (정확한 SAP contact 위해 quasi-static) ────────────────
DT, SUBSTEPS = 1e-3, 1            # implicit + SAP: dt=1ms OK
DURATION     = 10.0                # s
PLATE_VEL    = 1.0e-4             # 0.01 mm/s — quasi-static (SAP 수렴 안정)
                                  # d_max ≈ 19 μm = 0.5% strain on 4mm 두께
N_STEPS      = int(DURATION / DT)
RENDER_EVERY = 5                  # 2s/1ms = 2000 step → 400 frame @ 30fps ≈ 13s 영상
SAMPLE_EVERY = 5
TARGET_FACES = 5000                 # STL decimation: 원본 ~수천 face → 50 face
                                  # → tet/노드 대폭 축소 (테스트 반복용)

# 재료 (literature 정제 일반값)
E_TABLET   = 2.0e9    # 2 GPa
NU_TABLET  = 0.25
RHO_TABLET = 1300.0   # kg/m^3

# 정제 STL — repo 루트의 tablets_stl/stl 기준 (스크립트 위치로부터, 머신 독립)
_REPO_ROOT   = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TABLET_STL   = os.path.join(_REPO_ROOT, "tablets_stl", "stl", "tablet_R4.0_AR1.00_CV0.20.stl")
TABLET_SCALE = 1e-3   # mm → m

# Plate (시각 + 압축 driver)
PLATE_SIZE   = (0.025, 0.025, 0.003)
PLATE_MARGIN = 1.0e-6   # 1 μm — 초기 간극 (즉시 접촉 직전). 너무 크면 PLATE_VEL이
                        # 그 간극을 지나가는 데 시간이 걸려 F=0 구간만 길어짐.

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
    # P2 축 정렬: 이 STL은 두께(4mm)가 Y축, 지름(8mm)이 X·Z.
    # 두께를 압축축 Z로 보내기 위해 X축 +90° 회전 (Y→Z). 이후 bbox·배치 모두 올바른 축 기준.
    raw_mesh.apply_transform(tm.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
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
    gs.init(backend=gs.gpu, logging_level="warning", precision="64")  # SAP: 64bit 권장

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, 0)),
        # gravity=0: quasi-static 압축 검증. 중력 있으면 step당 9.8e-3 m/s 속도가
        # plate 에 누적돼 PLATE_VEL(1e-5) 보다 1000배 큰 노이즈가 SAP 임펄스 망침.
        # 정제 bot Dirichlet 으로 지지 중이라 중력 빼도 정역학 유지됨.
        fem_options=gs.options.FEMOptions(
            use_implicit_solver=True,         # backward Euler
            enable_vertex_constraints=True,   # bot 노드 Dirichlet 용 (top 은 contact)
            pcg_threshold=1e-10,              # 예제값
        ),
        coupler_options=gs.options.SAPCouplerOptions(  # *** Rigid–FEM 실제 접촉 ***
            pcg_threshold=1e-10,
            sap_convergence_atol=1e-10,
            sap_convergence_rtol=1e-10,
            linesearch_ftol=1e-10,
            rigid_rigid_contact_type="none",   # plate-plate 충돌 비활성 (불필요 + 버퍼 오버플로 회피)
        ),
        vis_options=gs.options.VisOptions(background_color=(0.95, 0.95, 0.97)),
        show_viewer=use_viewer,
    )

    # 정제 중심을 (0, 0, plate_gap/2) 에 위치 (gap 정중앙)
    z_center = plate_gap / 2
    # 하부 plate (정적, 정제 -z 면에서 PLATE_MARGIN 만큼 떨어짐).
    # bot 정제 노드는 vertex constraint 로 plate_bot 추종 → SAP contact 미사용 (시각 보조).
    plate_bot_z = -PLATE_MARGIN - PLATE_SIZE[2] / 2
    plate_bot = scene.add_entity(
        morph=gs.morphs.Box(size=PLATE_SIZE, pos=(0, 0, plate_bot_z), fixed=True),
        material=gs.materials.Rigid(),
        surface=gs.surfaces.Default(color=(0.45, 0.45, 0.55)),
    )
    # 상부 plate — SAP contact 로 정제 top 면 푸시. velocity control (PD) 로 -z 하강.
    # 디폴트 rho=600 이면 plate 질량 ~1g 라 contact impulse 한 번에 튀어 오름.
    # rho=1e7 (인공) → 약 18 kg ram → contact 외란 감쇠 + 안정 velocity tracking.
    plate_top_z0 = plate_gap + PLATE_MARGIN + PLATE_SIZE[2] / 2
    plate_top = scene.add_entity(
        morph=gs.morphs.Box(size=PLATE_SIZE, pos=(0, 0, plate_top_z0), fixed=False),
        material=gs.materials.Rigid(rho=1.0e7, coup_friction=1.0, friction=1.0),
        surface=gs.surfaces.Default(color=(0.70, 0.45, 0.45)),
    )
    # 정제 (FEM) — bbox center 를 (0, 0, z_center) 에 정확히 배치
    tablet_morph_pos = (tablet_pos[0], tablet_pos[1], tablet_pos[2] + z_center)
    tablet = scene.add_entity(
        material=gs.materials.FEM.Elastic(
            E=E_TABLET, nu=NU_TABLET, rho=RHO_TABLET, model="linear_corotated",
            friction_mu=1.0,  # SAP 마찰
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

    # 빌드 후 노드 위치 → BC 그룹 식별
    pos0 = _npy(tablet.get_state().pos).squeeze()  # (N, 3)
    z = pos0[:, 2]
    z_lo, z_hi = z.min(), z.max()
    band = (z_hi - z_lo) * 0.10
    top_idx = np.where(z > z_hi - band)[0].astype(int).tolist()
    bot_idx = np.where(z < z_lo + band)[0].astype(int).tolist()
    print(f"\n[tablet] nodes={len(pos0)}  z=[{z_lo*1000:.2f}, {z_hi*1000:.2f}] mm")
    print(f"[bc]     top nodes={len(top_idx)}, bot nodes={len(bot_idx)}  (band={band*1e3:.2f} mm)")

    # ── BC: bot 만 hard Dirichlet, top 은 SAP contact (vertex constraint 없음) ──
    tablet.set_vertex_constraints(bot_idx, link=plate_bot.links[0])   # 하면: 정적 추종
    print(f"[bc]     bot: hard Dirichlet (link=plate_bot)")
    print(f"[bc]     top: SAP contact with plate_top (no vertex constraint)")

    # SAP coupler 핸들 (반력 추출용)
    coupler = scene.sim.coupler
    top_idx_np = np.asarray(top_idx, dtype=np.int64)

    # ── plate_top: velocity control + force cap (real Crusher 모터 흉내) ──
    # 모델: "−PLATE_VEL 추종, 단 z 방향 최대 800 N (모터 peak)"
    #   F_PD = kv × (v_target − v_actual)        ← VELOCITY 모드
    #   F_actual = clamp(F_PD, force_range)      ← 마지막 step
    # kv 충분히 크게 → v_error 작아도 force_range 한계까지 즉시 saturate
    # 디폴트 force_range 는 ±∞ → kv 만 키우면 무한대 힘 → 비현실적
    n_dof = plate_top.n_dofs
    print(f"[plate_top] n_dofs={n_dof}")
    all_dofs = list(range(n_dof))
    plate_top.set_dofs_kp(np.zeros(n_dof))           # position 제어 OFF
    plate_top.set_dofs_kv(np.ones(n_dof) * 1.0e7)    # kv 1e7: v_err 1e-4 일 때 1e3 N → cap 으로 800 으로 잘림

    # force_range: z 방향만 ±800 N (Crusher 모터 peak). 나머지 DOF 는 ±∞ → kv 가 lock
    F_MAX_Z = 800.0
    fmin = np.full(n_dof, -np.inf)
    fmax = np.full(n_dof,  np.inf)
    if n_dof >= 3:
        fmin[2] = -F_MAX_Z
        fmax[2] =  F_MAX_Z
    plate_top.set_dofs_force_range(lower=fmin, upper=fmax)

    v_target = np.zeros(n_dof)
    if n_dof >= 3:
        v_target[2] = -PLATE_VEL                     # z 방향만 비영
    plate_top.control_dofs_velocity(velocity=v_target, dofs_idx_local=all_dofs)
    print(f"[plate_top] v_target_z={-PLATE_VEL} m/s   F_z ∈ [±{F_MAX_Z}] N   kv=1e7")

    def _read_F_top_z():
        """평판→정제 수직 반력 [N].
        SAP impulse on FEM top vertices (-z 성분의 합) / dt.
        평판이 -z 로 밀면 정제 top 노드는 -z 임펄스 받음 → F_on_tablet < 0.
        magnitude 를 반환 (압축력의 크기)."""
        imp = coupler.fem_state_v.impulse.to_numpy()   # (n_envs, n_verts, 3)
        if imp.ndim == 3:
            imp = imp[0]
        imp_z_top_sum = float(imp[top_idx_np, 2].sum())
        # 압축 시 imp_z < 0 → F_mag = -imp/dt > 0
        return -imp_z_top_sum / DT  # N (양수 = 압축력)

    # 데이터 기록
    t_arr, d_arr, top_z_arr, bot_z_arr, F_arr = [], [], [], [], []

    cam.start_recording()
    print("\n[run] stepping... (velocity control — set_pos 호출 없음)")
    for step in range(N_STEPS):
        t = step * DT

        # set_pos/set_dofs_velocity 호출 *없음*. control_dofs_velocity 가 build 시점에
        # 이미 PD target 으로 등록됐고, scene.step() 안에서 PD 가 매 step 자동 가동.
        scene.step()

        if step % SAMPLE_EVERY == 0:
            # plate 실제 z (PD 추종 결과 — 운동학 가정 d=v*t 가 아닌 측정값)
            plate_z_now = float(_npy(plate_top.get_pos())[2])
            d_actual = plate_top_z0 - plate_z_now    # 양수 = 하강량 [m]

            pos = _npy(tablet.get_state().pos).squeeze()
            top_z_now = float(pos[top_idx, 2].mean())
            bot_z_now = float(pos[bot_idx, 2].mean())
            F_now = _read_F_top_z()

            t_arr.append(t)
            d_arr.append(d_actual * 1e3)             # mm
            top_z_arr.append(top_z_now * 1e3)
            bot_z_arr.append(bot_z_now * 1e3)
            F_arr.append(F_now)

            if step % (SAMPLE_EVERY * 50) == 0:
                tab_h = (top_z_now - bot_z_now) * 1e3
                print(f"  t={t*1e3:7.1f} ms  d={d_actual*1e6:.2f} μm  top_z={top_z_now*1e3:.4f}mm  "
                      f"H={tab_h:.4f}mm  F={F_now:.3f} N")

        if step % RENDER_EVERY == 0:
            cam.render()

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved video] {MP4}")

    # ── Plot ───────────────────────────────────────────────────
    t_arr = np.array(t_arr); d_arr = np.array(d_arr)
    top_z_arr = np.array(top_z_arr); bot_z_arr = np.array(bot_z_arr)
    F_arr = np.array(F_arr)                          # N

    tablet_h = top_z_arr - bot_z_arr     # 정제 두께 [mm]
    H0 = tablet_h[0] if len(tablet_h) else 1
    eps = (H0 - tablet_h) / H0           # nominal strain
    sigma_MPa = E_TABLET * eps / 1e6     # nominal stress σ = E·ε [MPa]

    # W(t) = ∫ F · |v_actual| dt  사다리꼴 적분 [mJ]
    #   v_actual = d(plate_z)/dt  실측 (PD 추종 결과). PLATE_VEL 상수 가정 폐기.
    #   → plate 가 정지하면 W 도 plateau (정직한 측정).
    d_m = d_arr * 1e-3                               # mm → m
    v_actual = np.gradient(d_m, t_arr)               # m/s, 양수 = 하강 속도
    P_arr = F_arr * np.abs(v_actual)                 # 순간 일률 [W]
    W_arr = np.concatenate(([0.0], np.cumsum(0.5 * (P_arr[1:] + P_arr[:-1]) * np.diff(t_arr))))
    W_mJ = W_arr * 1e3

    fig, ax = plt.subplots(3, 2, figsize=(13, 12))

    ax[0,0].plot(t_arr, d_arr*1e3, 'r-', lw=1.5, label="$d$ (plate) [μm]")
    ax[0,0].plot(t_arr, (H0 - tablet_h)*1e3, 'b--', lw=1.5, label="$\\Delta H$ (tablet) [μm]")
    ax[0,0].set_xlabel("Time [s]"); ax[0,0].set_ylabel("Displacement [μm]")
    ax[0,0].set_title("(a) Plate vs tablet compression")
    ax[0,0].legend(); ax[0,0].grid(alpha=0.3)

    ax[0,1].plot(t_arr, eps*100, 'g-', lw=1.5)
    ax[0,1].set_xlabel("Time [s]"); ax[0,1].set_ylabel("Engineering strain $\\varepsilon$ [%]")
    ax[0,1].set_title("(b) Nominal compressive strain")
    ax[0,1].grid(alpha=0.3)

    ax[1,0].plot(t_arr, top_z_arr, 'r-', lw=1.5, label="top nodes")
    ax[1,0].plot(t_arr, bot_z_arr, 'b-', lw=1.5, label="bot nodes")
    ax[1,0].set_xlabel("Time [s]"); ax[1,0].set_ylabel("z [mm]")
    ax[1,0].set_title("(c) Tablet surface node tracking")
    ax[1,0].legend(); ax[1,0].grid(alpha=0.3)

    ax[1,1].plot(eps*100, sigma_MPa, 'k-', lw=1.5)
    ax[1,1].set_xlabel("Engineering strain $\\varepsilon$ [%]"); ax[1,1].set_ylabel("$\\sigma$ [MPa]")
    ax[1,1].set_title("(d) Nominal stress σ = E·ε")
    ax[1,1].axhspan(2, 3, color='r', alpha=0.12, label="tablet fracture ~2-3 MPa")
    ax[1,1].legend(); ax[1,1].grid(alpha=0.3)

    # (e) F(t) — SAP 반력
    ax[2,0].plot(t_arr, F_arr, 'm-', lw=1.5)
    ax[2,0].set_xlabel("Time [s]"); ax[2,0].set_ylabel("$F$ [N]")
    ax[2,0].set_title("(e) Reaction force on tablet  (SAP coupler impulse / dt)")
    ax[2,0].grid(alpha=0.3)

    # (f) W(t) — 누적 일
    ax[2,1].plot(t_arr, W_mJ, 'c-', lw=1.5)
    ax[2,1].set_xlabel("Time [s]"); ax[2,1].set_ylabel("$W$ [mJ]")
    ax[2,1].set_title("(f) Cumulative work  $W = \\int F\\cdot v\\,dt$")
    ax[2,1].grid(alpha=0.3)

    fig.suptitle(f"FEM Uniaxial (SAP Rigid–FEM contact) — {os.path.basename(TABLET_STL)}\n"
                 f"$E$={E_TABLET/1e9:.1f}GPa  $\\nu$={NU_TABLET}  $v$={PLATE_VEL*1e6:.2f}μm/s  "
                 f"$T$={DURATION:.1f}s")
    plt.tight_layout()
    plt.savefig(PNG, dpi=120, bbox_inches='tight')
    plt.close()
    print(f"[saved plot]  {PNG}")

    print(f"\n[summary] d_max={d_arr.max()*1e3:.2f} μm  ΔH_max={(H0-tablet_h.min())*1e3:.2f} μm  "
          f"ε_max={eps.max()*100:.3f}%")
    print(f"          F_max={F_arr.max():.3f} N   W_total={W_mJ[-1]:.4f} mJ")


if __name__ == "__main__":
    main(use_viewer=False)

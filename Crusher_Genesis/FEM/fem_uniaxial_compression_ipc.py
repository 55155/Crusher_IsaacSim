"""
fem_uniaxial_compression_ipc.py — IPC coupler 시험판 (최소·빠른 trend 확인용)

목표
----
IPC barrier가 plate–tablet *표면 사이 non-penetration* 을 강제하는지 trend 확인.
SAP 버전에서 plate가 정제 mesh를 *지나면서* contact 잃었던 문제(§4.6/§7-7)를
IPC barrier로 우회하는 가설 검증.

SAP 버전과의 차이 (필수만)
-------------------------
  - coupler_options: SAPCouplerOptions → IPCCouplerOptions
  - bot 정제 BC: vertex_constraints 사용 불가 (fem_entity.py:907 raise)
                 → bot plate를 fixed=True rigid 로 두고 IPC contact 로 지지
  - plate_top material: coup_type='two_way_soft_constraint' 명시
                        (Genesis PD가 driver, IPC가 barrier contact)
  - FEMOptions: enable_vertex_constraints 제거 (IPC 모드에서 무의미)
  - velocity control (PD + force_range) 그대로 유지

주의
----
  - pyuipc 필요 (없으면 import 단계에서 에러 — 친절한 메시지 출력)
  - IPC는 느림. 그래서 mesh·duration·plate margin 모두 최소로 잡음.
  - 정확한 F·W 값보다 *경향성*에 집중. F는 PD 명령식에서 역산
    (정밀 값은 IPC API 별도 조사 후 향후 작업).
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

# ── 옵션 (IPC 최소 trend 확인용) ────────────────
DT, SUBSTEPS = 1e-3, 1
DURATION     = 0.5                # s  — IPC 느려서 짧게
PLATE_VEL    = 5.0e-4             # 0.5 mm/s — 0.5s 안에 d_max=250 μm
N_STEPS      = int(DURATION / DT)
RENDER_EVERY = 5
SAMPLE_EVERY = 2
TARGET_FACES = 300                # IPC 비용 ↓ 위해 mesh 최소

# 재료
E_TABLET   = 2.0e9    # 2 GPa
NU_TABLET  = 0.25
RHO_TABLET = 1300.0   # kg/m^3

# 정제 STL
_REPO_ROOT   = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TABLET_STL   = os.path.join(_REPO_ROOT, "tablets_stl", "stl", "tablet_R4.0_AR1.00_CV0.20.stl")
TABLET_SCALE = 1e-3

# Plate
PLATE_SIZE   = (0.025, 0.025, 0.003)
PLATE_MARGIN = 1.5e-4    # 150 μm — IPC d_hat(100μm) 밖에서 시작해 barrier 초기 OFF

# IPC 파라미터
IPC_D_HAT      = 1.0e-4   # 100 μm — barrier 활성 거리. 정제 스케일 적합
IPC_RESISTANCE = 1.0e9    # 디폴트

# PD (SAP 버전과 동일)
PD_KV = 1.0e7
F_MAX_Z = 800.0           # Crusher 모터 peak

# 경로
_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "..", "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"fem_uniaxial_ipc_{_TS}.mp4")
PNG = os.path.join(OUT_DIR, f"fem_uniaxial_ipc_{_TS}.png")


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def main(use_viewer: bool = False):
    print("="*60)
    print(f" FEM Uniaxial Compression — IPC coupler  (viewer={use_viewer})")
    print("="*60)
    print(f"  E={E_TABLET/1e9:.1f} GPa  ν={NU_TABLET}  ρ={RHO_TABLET} kg/m³")
    print(f"  v_plate={PLATE_VEL*1e3:.3f} mm/s  duration={DURATION:.2f} s")
    print(f"  IPC d_hat={IPC_D_HAT*1e6:.0f} μm  resistance={IPC_RESISTANCE:.0e}")

    # libuipc 의존성 확인 — PIP 이름은 pyuipc, import 이름은 uipc (혼동 주의)
    try:
        import uipc  # noqa: F401
    except ImportError:
        sys.stderr.write(
            f"\n[ERROR] uipc (PIP 이름: pyuipc) 미설치. 현재 python: {sys.executable}\n"
            "  설치: python -m pip install pyuipc\n\n"
        )
        sys.exit(2)

    # ── STL load + decimate ──
    raw_mesh = tm.load(TABLET_STL)
    raw_mesh.apply_transform(tm.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    n_face_orig = len(raw_mesh.faces)
    if n_face_orig > TARGET_FACES:
        try:
            raw_mesh = raw_mesh.simplify_quadric_decimation(face_count=TARGET_FACES)
        except Exception as e:
            print(f"[decimate][warn] {e} — using original")
    n_face_new = len(raw_mesh.faces)
    STL_TMP = os.path.join(OUT_DIR, f"_tablet_ipc_{TARGET_FACES}.stl")
    raw_mesh.export(STL_TMP)
    print(f"[stl] decimate {n_face_orig} → {n_face_new} faces")

    bb = raw_mesh.bounding_box.bounds * TABLET_SCALE
    tab_lo, tab_hi = bb[0], bb[1]
    tab_h = float(tab_hi[2] - tab_lo[2])
    tablet_pos = (-0.5*(tab_lo[0]+tab_hi[0]),
                  -0.5*(tab_lo[1]+tab_hi[1]),
                  -0.5*(tab_lo[2]+tab_hi[2]))
    plate_gap = tab_h + 2*PLATE_MARGIN
    print(f"[stl] z thickness={tab_h*1e3:.2f} mm  plate_gap={plate_gap*1e3:.2f} mm")

    import genesis as gs
    # IPC 는 backend 제한 (Linux/Windows x86 CPU & Nvidia GPU). gs.gpu 시도, 안 되면 cpu.
    gs.init(backend=gs.gpu, logging_level="warning", precision="64")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, 0)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=IPC_D_HAT,
            contact_resistance=IPC_RESISTANCE,
            contact_friction_enable=False,         # uniaxial test 에 불필요
            enable_rigid_ground_contact=False,     # ground 없음
            enable_rigid_rigid_contact=False,      # plate–plate 충돌 불필요
            two_way_coupling=True,                 # IPC contact force → Genesis rigid 반영 (PD 추적용)
        ),
        # FEMOptions: IPC 모드에서 implicit·vertex_constraints 둘 다 무의미
        vis_options=gs.options.VisOptions(background_color=(0.95, 0.95, 0.97)),
        show_viewer=use_viewer,
    )

    # 하부 plate (정적, IPC contact 로 정제 bot 지지)
    z_center = plate_gap / 2
    plate_bot_z = -PLATE_MARGIN - PLATE_SIZE[2]/2
    plate_bot = scene.add_entity(
        morph=gs.morphs.Box(size=PLATE_SIZE, pos=(0, 0, plate_bot_z), fixed=True),
        material=gs.materials.Rigid(),     # coup_type=None → 비명시면 ipc_only 자동
        surface=gs.surfaces.Default(color=(0.45, 0.45, 0.55)),
    )
    # 상부 plate — velocity control 대상. coup_type='two_way_soft_constraint'
    # 명시: Genesis PD 가 driver, IPC 가 contact 처리, two-way coupling.
    plate_top_z0 = plate_gap + PLATE_MARGIN + PLATE_SIZE[2]/2
    plate_top = scene.add_entity(
        morph=gs.morphs.Box(size=PLATE_SIZE, pos=(0, 0, plate_top_z0), fixed=False),
        material=gs.materials.Rigid(
            rho=1.0e7,                                  # 무거운 ram
            coup_type='two_way_soft_constraint',        # IPC ↔ Genesis 양방향
        ),
        surface=gs.surfaces.Default(color=(0.70, 0.45, 0.45)),
    )
    # 정제 (FEM)
    tablet_morph_pos = (tablet_pos[0], tablet_pos[1], tablet_pos[2] + z_center)
    tablet = scene.add_entity(
        material=gs.materials.FEM.Elastic(
            E=E_TABLET, nu=NU_TABLET, rho=RHO_TABLET, model="linear_corotated",
        ),
        morph=gs.morphs.Mesh(file=STL_TMP, scale=TABLET_SCALE, pos=tablet_morph_pos),
    )

    # 카메라
    z_eye = plate_gap / 2
    cam = scene.add_camera(
        res=(960, 720),
        pos=(0.02, -0.20, z_eye),
        lookat=(0.0, 0.0, z_eye),
        fov=6, GUI=False,
    )

    scene.build(n_envs=0)

    # ── plate_top velocity control (SAP 버전과 동일 패턴) ──
    n_dof = plate_top.n_dofs
    all_dofs = list(range(n_dof))
    plate_top.set_dofs_kp(np.zeros(n_dof))
    plate_top.set_dofs_kv(np.ones(n_dof) * PD_KV)
    fmin = np.full(n_dof, -np.inf)
    fmax = np.full(n_dof,  np.inf)
    if n_dof >= 3:
        fmin[2] = -F_MAX_Z
        fmax[2] =  F_MAX_Z
    plate_top.set_dofs_force_range(lower=fmin, upper=fmax)

    v_target = np.zeros(n_dof)
    if n_dof >= 3:
        v_target[2] = -PLATE_VEL
    plate_top.control_dofs_velocity(velocity=v_target, dofs_idx_local=all_dofs)
    print(f"[plate_top] v_target_z={-PLATE_VEL} m/s  F_z ∈ ±{F_MAX_Z} N  kv={PD_KV:.0e}")

    # 빌드 후 정제 top/bot 노드 식별 (시각화·ε 계산용)
    pos0 = _npy(tablet.get_state().pos).squeeze()
    z = pos0[:, 2]
    z_lo, z_hi = z.min(), z.max()
    band = (z_hi - z_lo) * 0.10
    top_idx = np.where(z > z_hi - band)[0].astype(int)
    bot_idx = np.where(z < z_lo + band)[0].astype(int)
    print(f"[tablet] nodes={len(pos0)}  top={len(top_idx)}  bot={len(bot_idx)}")

    # 데이터 기록
    t_arr, plate_z_arr, plate_vz_arr, top_z_arr, bot_z_arr = [], [], [], [], []

    cam.start_recording()
    print("\n[run] stepping... (IPC barrier — 느릴 수 있음)")
    for step in range(N_STEPS):
        t = step * DT
        scene.step()

        if step % SAMPLE_EVERY == 0:
            plate_z_now = float(_npy(plate_top.get_pos())[2])
            # plate 실측 속도 (finite diff 대신 Genesis dofs_vel 직접)
            plate_vel = _npy(plate_top.get_dofs_velocity())
            plate_vz = float(plate_vel[2]) if len(plate_vel) >= 3 else 0.0

            pos = _npy(tablet.get_state().pos).squeeze()
            top_z = float(pos[top_idx, 2].mean())
            bot_z = float(pos[bot_idx, 2].mean())

            t_arr.append(t)
            plate_z_arr.append(plate_z_now * 1e3)        # mm
            plate_vz_arr.append(plate_vz)                # m/s
            top_z_arr.append(top_z * 1e3)
            bot_z_arr.append(bot_z * 1e3)

            if step % (SAMPLE_EVERY * 25) == 0:
                d_um = (plate_top_z0 - plate_z_now) * 1e6
                print(f"  t={t*1e3:6.1f} ms  d={d_um:7.2f} μm  v_z={plate_vz:+.2e} m/s  "
                      f"top_z={top_z*1e3:.4f}  bot_z={bot_z*1e3:.4f}")

        if step % RENDER_EVERY == 0:
            cam.render()

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved video] {MP4}")

    # ── Plot ──
    t_arr        = np.array(t_arr)
    plate_z_arr  = np.array(plate_z_arr)
    plate_vz_arr = np.array(plate_vz_arr)
    top_z_arr    = np.array(top_z_arr)
    bot_z_arr    = np.array(bot_z_arr)

    d_arr = plate_top_z0*1e3 - plate_z_arr               # mm — plate 하강량
    tablet_h = top_z_arr - bot_z_arr
    H0 = tablet_h[0] if len(tablet_h) else 1.0
    eps = (H0 - tablet_h) / H0                           # nominal strain

    # F 역산: PD 명령식 + force_range clamp
    # F_pd = kv × (v_target − v_actual), clamp by ±F_MAX_Z
    v_err = (-PLATE_VEL) - plate_vz_arr
    F_pd_raw  = PD_KV * v_err
    F_pd_clip = np.clip(F_pd_raw, -F_MAX_Z, F_MAX_Z)     # 실제 인가된 PD force [N]

    # contact reaction (≈ −F_pd in quasi-static, 부호 양수 = 압축력)
    F_contact_est = -F_pd_clip                            # plate에 가해진 contact F (Newton 3rd)

    # W = ∫ F · |v_plate| dt
    v_abs = np.abs(plate_vz_arr)
    P = F_contact_est * v_abs
    W_J = np.concatenate(([0.0], np.cumsum(0.5*(P[1:]+P[:-1]) * np.diff(t_arr))))
    W_mJ = W_J * 1e3

    fig, ax = plt.subplots(3, 2, figsize=(13, 12))

    ax[0,0].plot(t_arr, d_arr*1e3, 'r-', lw=1.5, label="$d$ (plate)")
    ax[0,0].plot(t_arr, (H0 - tablet_h)*1e3, 'b--', lw=1.5, label="$\\Delta H$ (tablet)")
    ax[0,0].set_xlabel("Time [s]"); ax[0,0].set_ylabel("Displacement [μm]")
    ax[0,0].set_title("(a) Plate vs tablet compression")
    ax[0,0].legend(); ax[0,0].grid(alpha=0.3)

    ax[0,1].plot(t_arr, plate_vz_arr*1e3, 'g-', lw=1.5, label="$v_z$ (plate, actual)")
    ax[0,1].axhline(-PLATE_VEL*1e3, color='k', ls=':', lw=1, label=f"target {-PLATE_VEL*1e3:.2f} mm/s")
    ax[0,1].set_xlabel("Time [s]"); ax[0,1].set_ylabel("plate $v_z$ [mm/s]")
    ax[0,1].set_title("(b) Plate velocity — IPC barrier 가 감속시키나?")
    ax[0,1].legend(); ax[0,1].grid(alpha=0.3)

    ax[1,0].plot(t_arr, top_z_arr, 'r-', lw=1.5, label="top nodes")
    ax[1,0].plot(t_arr, bot_z_arr, 'b-', lw=1.5, label="bot nodes")
    ax[1,0].set_xlabel("Time [s]"); ax[1,0].set_ylabel("z [mm]")
    ax[1,0].set_title("(c) Tablet surface node tracking")
    ax[1,0].legend(); ax[1,0].grid(alpha=0.3)

    ax[1,1].plot(t_arr, eps*100, 'k-', lw=1.5)
    ax[1,1].set_xlabel("Time [s]"); ax[1,1].set_ylabel("$\\varepsilon$ [%]")
    ax[1,1].set_title("(d) Engineering strain")
    ax[1,1].grid(alpha=0.3)

    ax[2,0].plot(t_arr, F_contact_est, 'm-', lw=1.5, label="$F_{contact}$ (estimate)")
    ax[2,0].axhline( F_MAX_Z, color='r', ls=':', lw=1, label=f"±{F_MAX_Z} N cap")
    ax[2,0].axhline(-F_MAX_Z, color='r', ls=':', lw=1)
    ax[2,0].set_xlabel("Time [s]"); ax[2,0].set_ylabel("$F$ [N]")
    ax[2,0].set_title("(e) Estimated reaction force  ($F_{pd}$ clamp 역산)")
    ax[2,0].legend(); ax[2,0].grid(alpha=0.3)

    ax[2,1].plot(t_arr, W_mJ, 'c-', lw=1.5)
    ax[2,1].set_xlabel("Time [s]"); ax[2,1].set_ylabel("$W$ [mJ]")
    ax[2,1].set_title("(f) Cumulative work  $W = \\int F\\cdot v\\,dt$")
    ax[2,1].grid(alpha=0.3)

    fig.suptitle(f"FEM Uniaxial — IPC coupler trend check  ({os.path.basename(TABLET_STL)})\n"
                 f"d_hat={IPC_D_HAT*1e6:.0f}μm   resistance={IPC_RESISTANCE:.0e}   "
                 f"$v$={PLATE_VEL*1e6:.0f}μm/s   $T$={DURATION:.2f}s   faces={TARGET_FACES}")
    plt.tight_layout()
    plt.savefig(PNG, dpi=120, bbox_inches='tight')
    plt.close()
    print(f"[saved plot]  {PNG}")

    d_max_um = d_arr.max() * 1e3
    print(f"\n[summary]  d_max={d_max_um:.2f} μm   ε_max={eps.max()*100:.4f}%   "
          f"F_max={F_contact_est.max():.2f} N   W_total={W_mJ[-1]:.4f} mJ")
    print(f"           plate v_z 평균={plate_vz_arr.mean():.2e} m/s  (target {-PLATE_VEL:.2e})")


if __name__ == "__main__":
    main(use_viewer=False)

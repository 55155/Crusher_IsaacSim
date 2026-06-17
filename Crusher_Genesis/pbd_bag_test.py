"""
pbd_bag_test.py
Samplebag_v2.stl 을 Genesis PBD.Cloth 로 시뮬 (CPU, headless).

목표:
  - 상단 실링부 2점 + 하단 실링부 2점 = 총 4점 고정
  - 헤드리스 N step 시뮬
  - 매 step 파티클 위치 통계 → 안정성 분석

안정성 지표:
  - NaN 입자 수
  - 평균/최대 변위 (drift)
  - 평균/최대 속도
  - 고정 입자의 drift (= 0 이어야 정상)
"""

import os, sys, time, math
import numpy as np
import torch
import genesis as gs
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ─────────────────────────────────────────────────────────────
STL_PATH = r"C:\Users\simuser\Downloads\Samplebag_v2.stl"

# STL 단위: mm → Genesis(m) 변환
SCALE       = 0.001         # 1 mm = 0.001 m
# STL bbox: X=[-10,10] Y=[-35,35] Z=[0,70]  (mm)
# → world: X=[-0.01,0.01], Y=[-0.035,0.035], Z=[0,0.07] (m)

POS         = (0.0, 0.0, 0.05)   # 봉투 중심을 z=5cm로 띄움 (낙하 여유)
EULER       = (0.0, 0.0, 0.0)

# 시뮬레이션
DT          = 5e-4
SUBSTEPS    = 10
SIM_TIME    = 2.0           # 2 초
LOG_EVERY   = 50            # 50 step (25ms) 마다 안정성 통계

# 출력
OUT_DIR     = os.path.join(os.path.dirname(__file__), "Sim_result")
os.makedirs(OUT_DIR, exist_ok=True)
PLOT_PATH   = os.path.join(OUT_DIR, "pbd_bag_stability.png")
CSV_PATH    = os.path.join(OUT_DIR, "pbd_bag_stability.csv")

# ─────────────────────────────────────────────────────────────


def find_corner_particles(pos_np):
    """
    pos_np : (N,3) 파티클의 world space 위치 [m]
    봉투 좌표계 가정 (scale 적용 후):
        X: 얇은 방향 (앞면/뒷면 두께)
        Y: 가로 (좌우 실링)
        Z: 세로 (상단=high, 하단=low)
    상단 / 하단 실링부 각각 2점씩 = 총 4점 반환.

    선택 규칙:
        top_left   : Z 상위 5% 중 Y 최소
        top_right  : Z 상위 5% 중 Y 최대
        bot_left   : Z 하위 5% 중 Y 최소
        bot_right  : Z 하위 5% 중 Y 최대
    """
    z   = pos_np[:, 2]
    y   = pos_np[:, 1]
    z_hi = np.quantile(z, 0.95)
    z_lo = np.quantile(z, 0.05)
    top_mask = z >= z_hi
    bot_mask = z <= z_lo

    top_idx = np.where(top_mask)[0]
    bot_idx = np.where(bot_mask)[0]

    top_left  = top_idx[np.argmin(y[top_idx])]
    top_right = top_idx[np.argmax(y[top_idx])]
    bot_left  = bot_idx[np.argmin(y[bot_idx])]
    bot_right = bot_idx[np.argmax(y[bot_idx])]

    return {
        "top_left"  : int(top_left),
        "top_right" : int(top_right),
        "bot_left"  : int(bot_left),
        "bot_right" : int(bot_right),
    }


def main():
    print("=" * 60)
    print(" PBD Bag stability test (CPU, headless)")
    print("=" * 60)

    if not os.path.exists(STL_PATH):
        sys.exit(f"[ERROR] STL not found: {STL_PATH}")

    gs.init(backend=gs.cpu, logging_level="warning")

    scene = gs.Scene(
        sim_options    = gs.options.SimOptions(
            dt       = DT,
            substeps = SUBSTEPS,
            gravity  = (0, 0, -9.81),
        ),
        pbd_options    = gs.options.PBDOptions(
            # 약포지 비닐 특성에 맞춘 iteration
            max_stretch_solver_iterations  = 25,
            max_bending_solver_iterations  = 8,
            max_volume_solver_iterations   = 3,
            max_density_solver_iterations  = 1,
            max_viscosity_solver_iterations= 1,
            particle_size = 2e-3,
        ),
        coupler_options = gs.options.LegacyCouplerOptions(
            rigid_pbd = True,
            rigid_mpm = False,
            rigid_sph = False,
            rigid_fem = False,
            mpm_sph   = False,
            mpm_pbd   = False,
            fem_mpm   = False,
            fem_sph   = False,
        ),
        show_viewer = False,
    )

    # 바닥 추가 (옵션 — 파티클이 떨어지지 않는지 시각 확인용)
    scene.add_entity(gs.morphs.Plane(),
                     material=gs.materials.Rigid())

    # ── PBD cloth 봉투 ───────────────────────────────────────
    bag = scene.add_entity(
        material = gs.materials.PBD.Cloth(),
        morph    = gs.morphs.Mesh(
            file  = STL_PATH,
            scale = SCALE,
            pos   = POS,
            euler = EULER,
        ),
    )

    print("[build] starting ...")
    t0 = time.time()
    scene.build(n_envs=0)
    print(f"[build] OK ({time.time()-t0:.1f}s)")

    # ── 초기 파티클 위치 확인 + 4 코너 식별 ──────────────────
    pos0 = bag.get_particles_pos()
    if hasattr(pos0, "cpu"):
        pos0 = pos0.cpu().numpy()
    pos0 = np.asarray(pos0)
    if pos0.ndim == 3:        # (n_envs, n_particles, 3) 케이스
        pos0 = pos0[0]
    N = pos0.shape[0]
    print(f"[mesh] particles = {N}")
    print(f"[mesh] bbox X={pos0[:,0].min():.4f}~{pos0[:,0].max():.4f}  "
                       f"Y={pos0[:,1].min():.4f}~{pos0[:,1].max():.4f}  "
                       f"Z={pos0[:,2].min():.4f}~{pos0[:,2].max():.4f}  [m]")

    corners = find_corner_particles(pos0)
    print("[fix] corner indices:")
    for k, idx in corners.items():
        p = pos0[idx]
        print(f"  {k:10s}  idx={idx:5d}  pos=({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f})")

    fix_idx = list(corners.values())
    bag.fix_particles(particles_idx_local=fix_idx)
    print(f"[fix] fixed {len(fix_idx)} particles")

    # ── 헤드리스 시뮬 + 안정성 로그 ──────────────────────────
    n_steps = int(SIM_TIME / DT)
    log_t   = []
    log_n_nan      = []
    log_disp_mean  = []
    log_disp_max   = []
    log_vel_mean   = []
    log_vel_max    = []
    log_fix_drift  = []   # 고정된 4점이 정말 안 움직이는지

    pos_prev = pos0.copy()
    print(f"\n[sim] dt={DT} substeps={SUBSTEPS}  total_steps={n_steps} ({SIM_TIME}s)")
    print(f"  {'t[s]':>6s} | {'NaN':>5s} | {'mean_d[mm]':>10s} | {'max_d[mm]':>10s} | "
          f"{'mean_v':>8s} | {'max_v':>8s} | {'fix_drift[mm]':>13s}")
    print("  " + "-"*78)

    sim_t0 = time.time()
    for k in range(n_steps):
        scene.step()

        if (k + 1) % LOG_EVERY == 0:
            pos = bag.get_particles_pos()
            if hasattr(pos, "cpu"):
                pos = pos.cpu().numpy()
            pos = np.asarray(pos)
            if pos.ndim == 3:
                pos = pos[0]

            n_nan = int(np.isnan(pos).any(axis=1).sum())
            disp  = np.linalg.norm(pos - pos0, axis=1)
            vel   = np.linalg.norm(pos - pos_prev, axis=1) / (LOG_EVERY * DT)
            fixed_drift = np.linalg.norm(pos[fix_idx] - pos0[fix_idx], axis=1).max()

            log_t.append((k+1) * DT)
            log_n_nan.append(n_nan)
            log_disp_mean.append(float(disp.mean()*1000))
            log_disp_max.append(float(disp.max()*1000))
            log_vel_mean.append(float(vel.mean()))
            log_vel_max.append(float(vel.max()))
            log_fix_drift.append(float(fixed_drift*1000))

            print(f"  {(k+1)*DT:6.3f} | {n_nan:5d} | "
                  f"{disp.mean()*1000:10.3f} | {disp.max()*1000:10.3f} | "
                  f"{vel.mean():8.4f} | {vel.max():8.4f} | "
                  f"{fixed_drift*1000:13.4f}")
            pos_prev = pos

    print(f"\n[sim] wall-time: {time.time()-sim_t0:.1f}s")

    # ── 안정성 판정 ──────────────────────────────────────────
    t = np.array(log_t)
    nans   = np.array(log_n_nan)
    d_mean = np.array(log_disp_mean)
    d_max  = np.array(log_disp_max)
    v_mean = np.array(log_vel_mean)
    v_max  = np.array(log_vel_max)
    drift  = np.array(log_fix_drift)

    print("\n" + "=" * 60)
    print(" 안정성 판정")
    print("=" * 60)
    verdict = "STABLE"
    if nans.max() > 0:
        verdict = "DIVERGED (NaN)"
    elif v_max[-1] > 100.0:
        verdict = "OSCILLATING (vel 발산)"
    elif d_max.max() > 1000.0:           # > 1m 이동
        verdict = "BLOWUP (displacement)"
    elif drift.max() > 5.0:              # 고정점이 5mm 이상 이동
        verdict = "FIX FAILED (corner drift)"

    print(f"  NaN 발생             : {nans.max()}")
    print(f"  최대 변위 (mm)        : {d_max.max():.3f}")
    print(f"  최종 평균 속도        : {v_mean[-1]:.4f}")
    print(f"  최종 최대 속도        : {v_max[-1]:.4f}")
    print(f"  고정점 최대 drift(mm) : {drift.max():.4f}")
    print(f"  ── VERDICT           : {verdict}")
    print("=" * 60)

    # ── 그래프 ───────────────────────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(f"PBD Bag stability — Samplebag_v2  N={N}  verdict={verdict}",
                 fontsize=11, fontweight="bold")

    axes[0].plot(t, d_mean, lw=1.2, label="mean displacement [mm]")
    axes[0].plot(t, d_max,  lw=1.2, label="max  displacement [mm]")
    axes[0].set_ylabel("displacement [mm]")
    axes[0].grid(True, alpha=0.3); axes[0].legend(fontsize=9)

    axes[1].plot(t, v_mean, lw=1.2, label="mean |v|")
    axes[1].plot(t, v_max,  lw=1.2, label="max  |v|")
    axes[1].set_ylabel("|v| [m/s]")
    axes[1].grid(True, alpha=0.3); axes[1].legend(fontsize=9)

    axes[2].plot(t, drift, color="tab:red", lw=1.5, label="fixed-corner drift [mm]")
    axes[2].axhline(5.0, color="gray", ls="--", lw=0.8, label="threshold 5 mm")
    axes[2].set_ylabel("fix drift [mm]"); axes[2].set_xlabel("t [s]")
    axes[2].grid(True, alpha=0.3); axes[2].legend(fontsize=9)

    fig.tight_layout()
    fig.savefig(PLOT_PATH, dpi=140, bbox_inches="tight")
    plt.close(fig)

    # ── CSV 저장 ────────────────────────────────────────────
    with open(CSV_PATH, "w", encoding="utf-8") as fh:
        fh.write("t_s,n_nan,disp_mean_mm,disp_max_mm,vel_mean,vel_max,fix_drift_mm\n")
        for i in range(len(t)):
            fh.write(f"{t[i]:.4f},{nans[i]},{d_mean[i]:.5f},{d_max[i]:.5f},"
                     f"{v_mean[i]:.5f},{v_max[i]:.5f},{drift[i]:.5f}\n")

    print(f"\n[saved] {PLOT_PATH}")
    print(f"[saved] {CSV_PATH}")


if __name__ == "__main__":
    main()

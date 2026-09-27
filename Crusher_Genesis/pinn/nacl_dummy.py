"""nacl_dummy.py — NaCl 가상 시나리오의 **더미 데이터** 생성기 (실측 아님)

모터 고장으로 실측이 없어서(2026-09-28) 파이프라인을 먼저 세우려고 만든 가짜 데이터다.
여기서 나온 숫자는 **어떤 물리적 주장에도 쓰면 안 된다.** 쓰임은 하나뿐이다:
"정답을 아는 데이터에서 PINN 이 커널을 복원하는가" — 방법 자체의 검증.

실제 실험과 같은 모양의 파일 세 종류를 만든다. 나중에 실측/실런으로 **파일만 바꿔
끼우면** features.py / train.py 는 그대로 돈다.

  loadcell_<lv>.csv  실측 대체  — 로드셀 슬라이더 반력 (t_s, crank_deg, F_N)
  sim_<lv>.npz       트윈 대체  — full_workflow crush npz 와 같은 키
                                  (fld_step, fld_fn_mag, dof_q, dt, fld_every)
  psd_<lv>.csv       실측 대체  — 체질 PSD (n, 구간별 질량분율)
  truth.json         정답 커널  — 복원 검증용 (실험에는 없는 파일)

설계 결정(docs/PINN.md §9)이 그대로 들어가 있다:
  * 무리 규모(W)는 **로드셀**이 정한다 — 실측 앵커.
  * 트윈은 **알별 몫(share)** 만 준다. 그래서 sim npz 의 절대값에는 일부러
    가짜 배리어 배율 K_BARRIER 를 곱해 둔다. 파이프라인이 절대값을 버리고
    비율만 쓰는지 이것으로 확인된다.
  * 대상은 정제가 아니라 **파우더**다. 초기 입도 600–850 µm.

실행:  python nacl_dummy.py        (출력: ./RESULT_dummy_nacl/)
"""
import json
import os

import numpy as np

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT_dummy_nacl")

# ── 기구 (docs/motor_spec.md) ────────────────────────────────────────────────
CRANK_R = 0.020                    # [m] 크랭크 반경
ROD_L = 0.080                      # [m] 커넥팅 로드
RPM = 8.0
OMEGA = RPM * 2 * np.pi / 60       # 0.8378 rad/s
T_STROKE = 2 * np.pi / OMEGA       # 7.5 s
DT = 5e-3                          # full_workflow 의 DT
FLD_EVERY = 10                     # 트윈 알별 기록 간격 [스텝] = 0.05 s

# ── 시료 — NaCl, 600–850 µm, 2 g ─────────────────────────────────────────────
MASS = 2.0e-3                      # [kg] 장입량
RHO_BULK = 1200.0                  # [kg/m3] 느슨한 충전 부피밀도 (더미)
AREA = 30e-3 * 30e-3               # [m2] 판 접촉 면적 (더미)
H0 = MASS / RHO_BULK / AREA        # [m] 초기 층 두께 ≈ 1.85 mm

# ── 운전 조건 = "어떤 힘으로" ────────────────────────────────────────────────
# 벽 간격으로 TDC 에서의 층 압축량 δ_max 를 정한다. 벽이 고정이므로 힘은 결과다.
LEVELS = {"low": 0.30e-3, "mid": 0.50e-3, "high": 0.65e-3}   # 첫 스트로크 δ_max [m]
N_STROKES = 40                     # "어떤 시간 동안" = 40 스트로크 = 5 분
PSD_AT = [0, 1, 2, 5, 10, 20, 40]  # 체질하는 스트로크 수

# ── 층 거동 (더미 구성식) ────────────────────────────────────────────────────
SIG0, EPS_C = 5e4, 0.12            # 하중: σ = SIG0 (exp(ε/EPS_C) − 1)
RHO_RES, Q_UNL = 0.6, 2.5          # 제하: 잔류 압축 δ_r = RHO_RES δ_max, 지수 Q_UNL
ETA_SET, S_SAT = 0.08, 0.12e-3     # 스트로크마다 영구 압밀 → 같은 벽 간격에서 힘이 준다
LC_NOISE = 1.5                     # [N] 로드셀 잡음

# ── 트윈 대체: 알별 하중 분배 ────────────────────────────────────────────────
N_GR = 500                         # 트윈 낟알 수
SHIELD = 0.25                      # 힘사슬에서 빠진(가려진) 알 비율
REARRANGE = 0.4                    # 스트로크마다 재배열되는 몫
Z_COORD = 3.0                      # Σ|f_i| / F_판  (내부 쌍이 중복 집계되므로 > 1)
K_BARRIER = 7.3                    # **가짜 배리어 배율** — 절대값이 틀렸다는 가정

# ── 정답 PBM (Vogel–Peukert 선택함수 + Austin 파쇄분포) ─────────────────────
EDGES_UM = np.array([850, 600, 425, 300, 212, 150, 106, 75, 53, 0], float)  # √2 체
FMAT = 10.0                        # [kg/(J·m)]
EMIN_REF, X_REF = 5.0, 725e-6      # e_min(x) = EMIN_REF · X_REF / x  → 작을수록 단단 (분쇄 한계)
AUSTIN = dict(phi=0.40, gamma=0.90, beta=4.0)
PSD_NOISE_ABS, PSD_NOISE_REL = 0.005, 0.03


def class_sizes(edges_um=EDGES_UM):
    """구간 대표 입도 [m]. 마지막(<53 µm) 구간은 53/√2."""
    e = edges_um
    x = np.sqrt(e[:-2] * e[1:-1])
    return np.append(x, e[-2] / np.sqrt(2)) * 1e-6


def austin_b(phi, gamma, beta, edges_um=EDGES_UM):
    """b[i, j] = 부모 구간 j 가 깨질 때 구간 i 로 가는 질량분율. Σ_i b_ij = 1."""
    K = len(edges_um) - 1
    B = lambda y: phi * y ** gamma + (1 - phi) * y ** beta
    b = np.zeros((K, K))
    for j in range(K - 1):
        lo = edges_um[j + 1]
        for i in range(j + 1, K):
            up = B(edges_um[i] / lo)
            dn = B(edges_um[i + 1] / lo) if i < K - 1 else 0.0
            b[i, j] = up - dn
    return b


def slider_x(theta):
    """크랭크각 → 슬라이더 위치 [m]. θ=0 이 TDC(벽에 가장 가까움)."""
    return CRANK_R * np.cos(theta) + np.sqrt(ROD_L ** 2 - (CRANK_R * np.sin(theta)) ** 2)


def stroke_force(theta, dmax):
    """한 스트로크의 슬라이더 반력 [N]. θ 는 (−π, π], dmax 는 그 스트로크의 TDC 압축량."""
    d = dmax - (CRANK_R + ROD_L - slider_x(theta))           # 층 압축량
    F = np.zeros_like(theta)
    Fmax = SIG0 * (np.exp(dmax / H0 / EPS_C) - 1) * AREA
    load = (theta <= 0) & (d > 0)
    F[load] = SIG0 * (np.exp(d[load] / H0 / EPS_C) - 1) * AREA
    dr = RHO_RES * dmax
    unl = (theta > 0) & (d > dr)
    F[unl] = Fmax * ((d[unl] - dr) / (dmax - dr)) ** Q_UNL
    return F


def vp_selection(x, e):
    """Vogel–Peukert: 비에너지 e [J/kg] 를 받은 입도 x 입자의 파쇄확률."""
    emin = EMIN_REF * X_REF / x
    return 1 - np.exp(-FMAT * x * np.clip(e - emin, 0, None))


def stroke_work(theta, F):
    s = slider_x(theta)
    ds = np.diff(s)
    Fm = 0.5 * (F[1:] + F[:-1])
    w_load = float(np.sum(Fm[ds > 0] * ds[ds > 0]))
    w_unl = float(np.sum(Fm[ds < 0] * -ds[ds < 0]))
    return w_load, w_unl


def make_level(lv, dmax0, rng, b_true, x):
    steps_per = int(round(T_STROKE / DT))
    t_all, th_all, F_all = [], [], []
    fld_step, fld_mag, fld_q = [], [], []
    w = rng.exponential(1.0, N_GR)
    w[rng.random(N_GR) < SHIELD] *= 0.05
    s_perm = 0.0
    m = np.zeros(len(x)); m[0] = 1.0
    psd = {0: m.copy()}
    S_hist, W_hist = [], []
    for n in range(N_STROKES):
        dmax = dmax0 - s_perm
        k = np.arange(steps_per)
        th = -np.pi + OMEGA * k * DT
        F = stroke_force(th, dmax)
        t = (n * steps_per + k) * DT
        t_all.append(t); th_all.append(th); F_all.append(F)

        # 트윈 대체 — 알별 법선력 크기합. 몫 w 를 따라 판 힘을 나눠 갖는다.
        if n:
            w_new = rng.exponential(1.0, N_GR)
            w_new[rng.random(N_GR) < SHIELD] *= 0.05
            w = (1 - REARRANGE) * w + REARRANGE * w_new
        kk = k[::FLD_EVERY]
        f_i = (K_BARRIER * Z_COORD * F[kk, None] * (w / w.sum())[None, :]
               * (1 + 0.05 * rng.standard_normal((len(kk), N_GR))))
        fld_step.append(n * steps_per + kk)
        fld_mag.append(np.clip(f_i, 0, None).astype(np.float32))
        fld_q.append(np.stack([th[kk], np.full(len(kk), dmax)], 1))

        # 정답 PBM — 알별 비에너지 분포 위에서 VP 확률을 평균한다.
        wl, _ = stroke_work(th, F)
        share = w / w.sum()
        e_k = wl / MASS * share * N_GR                        # 평균 = W/M
        S = vp_selection(x[:, None], e_k[None, :]).mean(1)
        S[-1] = 0.0
        m = m - S * m + b_true @ (S * m)
        S_hist.append(S.tolist()); W_hist.append(wl)
        if n + 1 in PSD_AT:
            psd[n + 1] = m.copy()
        s_perm += ETA_SET * dmax * (1 - s_perm / S_SAT)

    t = np.concatenate(t_all); th = np.concatenate(th_all); F = np.concatenate(F_all)
    Fn = F + LC_NOISE * rng.standard_normal(len(F))
    with open(os.path.join(OUT, f"loadcell_{lv}.csv"), "w", encoding="utf-8") as fh:
        fh.write("# DUMMY — 실측 아님 (nacl_dummy.py)\nt_s,crank_deg,F_N\n")
        for a, bb, c in zip(t, np.degrees(th), Fn):
            fh.write(f"{a:.4f},{bb:.4f},{c:.4f}\n")

    np.savez_compressed(
        os.path.join(OUT, f"sim_{lv}.npz"),
        fld_step=np.concatenate(fld_step).astype(np.int32),
        fld_fn_mag=np.concatenate(fld_mag),
        dof_q=np.concatenate(fld_q).astype(np.float32),
        dt=DT, fld_every=FLD_EVERY, n_grains=N_GR, DUMMY=True)

    cols = [f"{EDGES_UM[i+1]:.0f}-{EDGES_UM[i]:.0f}" for i in range(len(x))]
    with open(os.path.join(OUT, f"psd_{lv}.csv"), "w", encoding="utf-8") as fh:
        fh.write("# DUMMY — 실측 아님 (nacl_dummy.py). 질량분율, 구간 µm\n")
        fh.write("n," + ",".join(cols) + "\n")
        for n, mm in psd.items():
            y = mm * (1 + PSD_NOISE_REL * rng.standard_normal(len(mm))) \
                + PSD_NOISE_ABS * rng.standard_normal(len(mm)) * (n > 0)
            y = np.clip(y, 0, None); y /= y.sum()
            fh.write(f"{n}," + ",".join(f"{v:.5f}" for v in y) + "\n")
    return dict(S=S_hist, W=W_hist, psd_clean={int(k): v.tolist() for k, v in psd.items()},
                F_peak_first=float(F_all[0].max()), F_peak_last=float(F_all[-1].max()))


def main():
    os.makedirs(OUT, exist_ok=True)
    rng = np.random.default_rng(20260928)
    x = class_sizes()
    b_true = austin_b(**AUSTIN)
    truth = dict(note="DUMMY 정답 — 실험에는 없는 파일", austin=AUSTIN, fmat=FMAT,
                 emin_ref=EMIN_REF, x_ref=X_REF, k_barrier=K_BARRIER, mass=MASS,
                 edges_um=EDGES_UM.tolist(), levels={})
    for lv, d0 in LEVELS.items():
        r = make_level(lv, d0, rng, b_true, x)
        truth["levels"][lv] = dict(dmax0=d0, **r)
        print(f"[dummy] {lv:4s}  δ0={d0*1e3:.2f}mm  F_peak {r['F_peak_first']:.0f}→"
              f"{r['F_peak_last']:.0f} N  W₁={r['W'][0]*1e3:.1f} mJ  "
              f"600–850 잔류(40회)={r['psd_clean'][40][0]:.3f}")
    with open(os.path.join(OUT, "truth.json"), "w", encoding="utf-8") as fh:
        json.dump(truth, fh, ensure_ascii=False, indent=1)
    print(f"[dummy] → {OUT}")


if __name__ == "__main__":
    main()

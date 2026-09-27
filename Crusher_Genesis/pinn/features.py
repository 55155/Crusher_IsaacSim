"""features.py — 로드셀 + 트윈 npz → 스트로크별 특징 φ_n

설계 원칙(docs/PINN.md §9):
  * 무리 규모 W 는 **로드셀**에서만 뽑는다. 트윈의 절대값은 쓰지 않는다.
  * 트윈에서는 알별 **몫** share_i = I_i / Σ I_j 만 쓴다 (I_i = 스트로크 동안
    알 i 의 법선력 크기합 적분). 절대 배율은 여기서 나눗셈으로 사라진다.

입력 파일은 nacl_dummy.py 가 만든 더미든, 실측 로드셀 CSV / full_workflow 의
crush_<TAG>.npz 든 모양이 같으면 그대로 읽는다.
"""
import numpy as np

CRANK_R, ROD_L = 0.020, 0.080


def slider_x(theta):
    return CRANK_R * np.cos(theta) + np.sqrt(ROD_L ** 2 - (CRANK_R * np.sin(theta)) ** 2)


def stroke_index(theta):
    """크랭크각 [rad] → 스트로크 번호. 후퇴점(−π)을 지날 때 넘어간다."""
    return np.floor((np.unwrap(theta) + np.pi) / (2 * np.pi) + 1e-5).astype(int)


def loadcell_features(csv_path):
    """스트로크마다 W(하중 일), W_irr(비가역 일), F_peak(참고), τ(접촉 시간)."""
    d = np.genfromtxt(csv_path, delimiter=",", skip_header=1, names=True, encoding="utf-8")
    th = np.radians(d["crank_deg"]); F = d["F_N"]; t = d["t_s"]
    idx = stroke_index(th)
    out = {k: [] for k in ("W", "W_irr", "F_peak", "tau")}
    for n in np.unique(idx):
        sel = idx == n
        s, f = slider_x(th[sel]), F[sel]
        ds, fm = np.diff(s), 0.5 * (f[1:] + f[:-1])
        w_load = np.sum(fm[ds > 0] * ds[ds > 0])
        w_unl = np.sum(fm[ds < 0] * -ds[ds < 0])
        on = f > max(3.0, 0.02 * f.max())                 # 잡음 바닥 위를 접촉으로 본다
        dt = np.median(np.diff(t[sel]))
        out["W"].append(w_load); out["W_irr"].append(w_load - w_unl)
        out["F_peak"].append(f.max()); out["tau"].append(on.sum() * dt)
    return {k: np.asarray(v) for k, v in out.items()}


def share_features(npz_path):
    """트윈 알별 몫. 반환 share: (스트로크, 낟알), 행 합 = 1."""
    z = np.load(npz_path)
    mag = z["fld_fn_mag"].astype(np.float64)
    idx = stroke_index(z["dof_q"][:, 0].astype(np.float64))
    shares = []
    for n in np.unique(idx):
        I = mag[idx == n].sum(0)
        shares.append(I / I.sum())
    return np.asarray(shares)


def stroke_features(csv_path, npz_path, mass):
    lc = loadcell_features(csv_path)
    sh = share_features(npz_path)
    n = min(len(lc["W"]), len(sh))
    e_norm = sh[:n] * sh.shape[1]                          # 알별 비에너지 / 평균
    return dict(
        W=lc["W"][:n], W_irr=lc["W_irr"][:n], F_peak=lc["F_peak"][:n], tau=lc["tau"][:n],
        e_mean=lc["W"][:n] / mass,                         # [J/kg] 로드셀 앵커
        share=sh[:n],
        share_cv=e_norm.std(1), share_p90=np.quantile(e_norm, 0.9, axis=1))

"""train.py — NaCl 가상 시나리오: PBM × PINN 학습과 복원 검증

구조 (docs/PINN.md §5, §9):

    로드셀 → e_mean,n = W_n / M          (무리 비에너지, 실측 앵커)
    트윈   → e_norm,k,n = share_k · N     (알별 비에너지 / 평균, 배율 무관)
    NN_θ(log x, log e) → P(x, e)          단일 입자 파쇄확률 (e·x 에 단조 증가)
    S_i,n = mean_k P(x_i, e_mean,n · e_norm,k,n)       ← 트윈 분포 위에서 평균
    b     = Austin(φ, γ, β)               Σ_i b_ij = 1 이 구조로 성립
    m_n+1 = m_n − S∘m_n + b (S∘m_n)       스트로크 단위 이산 PBM (질량보존 정확)

학습: low + high 두 하중 수준의 PSD. 검증: mid (학습에 안 쓴 하중).
비교: 트윈 분포를 버리고 e_mean 만 쓰는 평균장 모델(ablation).

**더미 데이터 위의 결과다.** 복원이 된다는 것은 "이 측정 설계로 커널이 식별된다"는
뜻이지 NaCl 의 물성에 대한 주장이 아니다.

실행:  python train.py      (먼저 python nacl_dummy.py)
"""
import json
import os
import sys

import numpy as np
import torch
import torch.nn as nn

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from features import stroke_features            # noqa: E402
from nacl_dummy import (OUT, MASS, EDGES_UM, class_sizes, austin_b,  # noqa: E402
                        vp_selection)

torch.manual_seed(0)
DEV = "cuda" if torch.cuda.is_available() else "cpu"
TRAIN, TEST = ("low", "high"), ("mid",)
LV = TRAIN + TEST
X = class_sizes()
K = len(X)
E_REF = 30.0                                    # [J/kg] 입력 정규화용
X_REF = 725e-6


def load_psd(lv):
    d = np.genfromtxt(os.path.join(OUT, f"psd_{lv}.csv"), delimiter=",", skip_header=2, encoding="utf-8")
    return d[:, 0].astype(int), d[:, 1:]


def passing(m):
    """체 눈금 EDGES_UM[1:-1] 에서의 통과분율. m: (..., K)."""
    rev = torch.flip(torch.cumsum(torch.flip(m, [-1]), -1), [-1])   # Σ_{k≥i} m_k
    return rev[..., 1:]


def d50(m):
    """질량 중앙 입도 [µm] — 로그 입도 보간."""
    edges = EDGES_UM[:-1]                                           # 850 … 53
    P = np.concatenate([[1.0], np.flip(np.cumsum(np.flip(m)))[1:]])
    for a in range(len(P) - 1):
        if P[a] >= 0.5 > P[a + 1]:
            f = (P[a] - 0.5) / (P[a] - P[a + 1])
            return float(np.exp(np.log(edges[a]) + f * (np.log(edges[a + 1]) - np.log(edges[a]))))
    return float(edges[-1])


class Kernel(nn.Module):
    def __init__(self, use_dist=True):
        super().__init__()
        self.use_dist = use_dist
        self.net = nn.Sequential(nn.Linear(2, 32), nn.Tanh(), nn.Linear(32, 32), nn.Tanh(),
                                 nn.Linear(32, 1))
        self.a = nn.Parameter(torch.tensor([0.0, 0.2, 1.0]))      # φ, γ, β 의 원시값

    def P(self, logx, loge):
        return torch.sigmoid(self.net(torch.stack([logx, loge], -1)).squeeze(-1) - 2.0)

    def austin(self):
        phi = torch.sigmoid(self.a[0])
        gam = nn.functional.softplus(self.a[1]) + 0.1
        bet = nn.functional.softplus(self.a[2]) + gam                # β > γ
        E = torch.tensor(EDGES_UM, dtype=torch.float32, device=DEV)
        b = torch.zeros(K, K, device=DEV)
        for j in range(K - 1):
            y = E[j + 1:K] / E[j + 1]                               # 구간 상단 / 부모 하단
            B = phi * y ** gam + (1 - phi) * y ** bet
            B = torch.cat([B, torch.zeros(1, device=DEV)])
            b[j + 1:, j] = B[:-1] - B[1:]
        return b, (phi, gam, bet)

    def selection(self, e_mean, e_norm):
        """e_mean: (Ns,), e_norm: (Ns, N) → S: (Ns, K)."""
        logx = torch.log(torch.tensor(X / X_REF, dtype=torch.float32, device=DEV))
        if self.use_dist:
            e = e_mean[:, None] * e_norm                             # (Ns, N)
        else:
            e = e_mean[:, None]
        le = torch.log(e.clamp_min(1e-3) / E_REF)
        Ns, Ng = le.shape
        lx = logx[None, :, None].expand(Ns, K, Ng)
        S = self.P(lx, le[:, None, :].expand(Ns, K, Ng)).mean(-1)
        return S * torch.tensor([1.0] * (K - 1) + [0.0], device=DEV)  # 최세 구간은 안 깨진다


def rollout(model, feat):
    b, _ = model.austin()
    S = model.selection(feat["e_mean"], feat["e_norm"])
    m = torch.zeros(K, device=DEV); m[0] = 1.0
    ms = [m]
    for n in range(S.shape[0]):
        sm = S[n] * m
        m = m - sm + b @ sm
        ms.append(m)
    return torch.stack(ms), S


def monotone_penalty(model):
    """∂P/∂log e ≥ 0, ∂P/∂log x ≥ 0 — 더 세게 칠수록, 클수록 잘 깨진다(분쇄 한계)."""
    lx = (torch.rand(2048, device=DEV) * 3.5 - 3.0).requires_grad_()
    le = (torch.rand(2048, device=DEV) * 5.0 - 3.0).requires_grad_()
    gx, ge = torch.autograd.grad(model.P(lx, le).sum(), (lx, le), create_graph=True)
    return (torch.relu(-gx) ** 2).mean() + (torch.relu(-ge) ** 2).mean()


def fit(feats, psd, use_dist, iters=2500):
    model = Kernel(use_dist).to(DEV)
    opt = torch.optim.Adam(model.parameters(), lr=5e-3)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, iters)
    for it in range(iters):
        loss = 0.0
        for lv in TRAIN:
            ms, _ = rollout(model, feats[lv])
            ns, obs = psd[lv]
            loss = loss + ((passing(ms[ns]) - passing(obs)) ** 2).mean()
        loss = loss + 1e-2 * monotone_penalty(model)
        opt.zero_grad(); loss.backward(); opt.step(); sched.step()
        if it % 500 == 0 or it == iters - 1:
            print(f"  [{'분포' if use_dist else '평균장'}] it {it:4d}  loss {loss.item():.3e}")
    return model


def main():
    feats, psd, raw = {}, {}, {}
    for lv in LV:
        f = stroke_features(os.path.join(OUT, f"loadcell_{lv}.csv"),
                            os.path.join(OUT, f"sim_{lv}.npz"), MASS)
        raw[lv] = f
        feats[lv] = dict(
            e_mean=torch.tensor(f["e_mean"], dtype=torch.float32, device=DEV),
            e_norm=torch.tensor(f["share"] * f["share"].shape[1], dtype=torch.float32,
                                device=DEV))
        ns, obs = load_psd(lv)
        psd[lv] = (torch.tensor(ns, device=DEV), torch.tensor(obs, dtype=torch.float32,
                                                                device=DEV))
        print(f"[feat] {lv:4s}  스트로크 {len(f['W'])}  W₁={f['W'][0]*1e3:.1f} mJ  "
              f"e₁={f['e_mean'][0]:.1f} J/kg  W_irr/W={f['W_irr'][0]/f['W'][0]:.2f}  "
              f"share CV={f['share_cv'][0]:.2f}")

    truth = json.load(open(os.path.join(OUT, "truth.json"), encoding="utf-8"))
    models = {k: fit(feats, psd, use_dist=k == "dist") for k in ("dist", "mean")}

    report = {}
    with torch.no_grad():
        for name, mdl in models.items():
            _, (phi, gam, bet) = mdl.austin()
            r = dict(austin=dict(phi=phi.item(), gamma=gam.item(), beta=bet.item()), lv={})
            for lv in LV:
                ms, S = rollout(mdl, feats[lv])
                ms = ms.cpu().numpy()
                clean = np.array([truth["levels"][lv]["psd_clean"][str(n)]
                                  for n in psd[lv][0].cpu().numpy()])
                pred = ms[psd[lv][0].cpu().numpy()]
                rmse = float(np.sqrt(((passing(torch.tensor(pred)) -
                                       passing(torch.tensor(clean))) ** 2).mean()))
                r["lv"][lv] = dict(
                    rmse_passing_vs_truth=rmse,
                    d50_pred=[d50(m) for m in ms],
                    d50_true=[d50(np.array(truth["levels"][lv]["psd_clean"][str(n)]))
                              for n in psd[lv][0].cpu().numpy()],
                    S1_pred=S[0].cpu().numpy().tolist(),
                    S1_true=truth["levels"][lv]["S"][0])
            report[name] = r

    print("\n[결과] 통과분율 RMSE (정답 대비, 잡음 없는 참값)")
    for name in models:
        tag = "트윈 분포 사용" if name == "dist" else "평균장(e_mean 만)"
        row = "  ".join(f"{lv}{'*' if lv in TEST else ''} {report[name]['lv'][lv]['rmse_passing_vs_truth']:.4f}"
                        for lv in LV)
        a = report[name]["austin"]
        print(f"  {tag:14s} {row}   Austin φ={a['phi']:.2f} γ={a['gamma']:.2f} β={a['beta']:.2f}")
    print(f"  정답 Austin φ={truth['austin']['phi']} γ={truth['austin']['gamma']} "
          f"β={truth['austin']['beta']}   (* = 학습에 안 쓴 하중)")
    json.dump(report, open(os.path.join(OUT, "report.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)

    plot(raw, psd, report, models, truth)


def plot(raw, psd, report, models, truth):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "Malgun Gothic", "axes.unicode_minus": False,
                         "font.size": 10})
    from features import slider_x
    col = {"low": "#0072B2", "mid": "#D55E00", "high": "#009E73"}
    fig, ax = plt.subplots(2, 2, figsize=(12, 9))

    # (a) 로드셀 F–s 루프 — 무리 규모의 실측 앵커
    a = ax[0, 0]
    for lv in LV:
        d = np.genfromtxt(os.path.join(OUT, f"loadcell_{lv}.csv"), delimiter=",",
                          skip_header=1, names=True, encoding="utf-8")
        th = np.radians(d["crank_deg"]); n_per = int(round(7.5 / 5e-3))
        for n, ls in ((0, "-"), (39, ":")):
            sl = slice(n * n_per, (n + 1) * n_per)
            s = (0.1 - slider_x(th[sl])) * 1e3
            a.plot(-s, d["F_N"][sl], ls, color=col[lv], lw=1.6,
                   label=f"{lv} {'1' if n == 0 else '40'}회째")
    a.set_xlim(-2.5, 0.05); a.set_xlabel("슬라이더 위치 - TDC [mm]"); a.set_ylabel("로드셀 반력 [N]")
    a.set_title("(a) 로드셀 F–s 루프 — 면적 = 스트로크당 일 W", loc="left")
    a.legend(fontsize=8, ncol=2); a.grid(alpha=.3)

    # (b) 트윈 알별 하중 — 절대값(가짜 배리어 배율) vs 몫
    a = ax[0, 1]
    z = np.load(os.path.join(OUT, "sim_mid.npz"))
    frame = int(np.argmax(z["fld_fn_mag"].sum(1)[:150]))
    absF = z["fld_fn_mag"][frame]
    e_norm = raw["mid"]["share"][0] * raw["mid"]["share"].shape[1]
    a.hist(e_norm, bins=40, color=col["mid"], alpha=.8)
    a.set_xlabel("알별 비에너지 / 평균  (share · N)"); a.set_ylabel("낟알 수")
    a.set_title("(b) 트윈이 주는 것은 분포 — 절대값은 버린다", loc="left")
    a.text(0.97, 0.95, f"트윈 절대값 합 {absF.sum():.0f} N\n로드셀 {raw['mid']['F_peak'][0]:.0f} N\n"
           f"→ 배율 {absF.sum()/raw['mid']['F_peak'][0]:.1f}× (배리어·중복집계)\n나눗셈으로 소거",
           transform=a.transAxes, ha="right", va="top", fontsize=9,
           bbox=dict(fc="white", ec="#999"))
    a.grid(alpha=.3)

    # (c) 학습된 단일입자 파쇄확률 vs 정답
    a = ax[1, 0]
    e = np.logspace(0, 2.5, 200)
    mdl = models["dist"]
    for x_um, c in ((725, "#000"), (357, "#555"), (178, "#999")):
        a.plot(e, vp_selection(x_um * 1e-6, e), "-", color=c, lw=2, label=f"정답 {x_um} µm")
        with torch.no_grad():
            pp = mdl.P(torch.full((200,), np.log(x_um * 1e-6 / X_REF), device=DEV),
                       torch.tensor(np.log(e / E_REF), dtype=torch.float32, device=DEV))
        a.plot(e, pp.cpu().numpy(), "--", color=c, lw=2, label=f"PINN {x_um} µm")
    a.set_xscale("log"); a.set_xlabel("알이 받은 비에너지 e [J/kg]"); a.set_ylabel("스트로크당 파쇄확률 P")
    a.set_title("(c) NN_θ 가 복원한 파쇄 법칙 (실선 정답 / 점선 PINN)", loc="left")
    a.legend(fontsize=8, ncol=2); a.grid(alpha=.3)

    # (d) d50(n) — 학습 2수준 + 검증 1수준
    a = ax[1, 1]
    for lv in LV:
        r = report["dist"]["lv"][lv]; rm = report["mean"]["lv"][lv]
        n = np.arange(len(r["d50_pred"]))
        a.plot(n, r["d50_pred"], "-", color=col[lv], lw=2,
               label=f"{lv}{' (검증)' if lv in TEST else ''} — PINN")
        a.plot(n, rm["d50_pred"], ":", color=col[lv], lw=1.4)
        ns, obs = psd[lv]
        a.plot(ns.cpu().numpy(), [d50(o) for o in obs.cpu().numpy()], "o", color=col[lv],
               mfc="white" if lv in TEST else col[lv], ms=6)
    a.set_yscale("log"); a.set_xlabel("스트로크 수 n"); a.set_ylabel("d50 [µm]")
    a.set_title("(d) d50(n) — 선: PINN(트윈 분포), 점선: 평균장, ○: 더미 체질", loc="left")
    a.legend(fontsize=8); a.grid(alpha=.3, which="both")

    fig.suptitle("NaCl 가상 시나리오 (DUMMY 데이터) — 600–850 µm, 2 g, 8 RPM × 40회",
                 fontsize=13, fontweight="bold")
    fig.tight_layout()
    p = os.path.join(OUT, "nacl_dummy_pinn.png")
    fig.savefig(p, dpi=130)
    print(f"[plot] → {p}")


if __name__ == "__main__":
    main()

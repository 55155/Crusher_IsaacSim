"""
plot_reaction_profiles.py — 실기 ForceGage 반력 프로파일을 각도별로 흰 배경 시계열 플롯.

입력 : Real_result/반력프로파일/{θ}deg.txt (파이프구분) 또는 {θ}deg*.xls (ForceGage export)
출력 : Real_result/plots/{θ}deg_reaction_profile.png  (흰 배경, 비전문가용 제목, 큰 텍스트)
"""
import os, re, glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "Real_result", "반력프로파일")
OUT  = os.path.join(HERE, "Real_result", "plots")
ANGLES = [0, 30, 45, 60, 75, 90, 105, 120]
CROP_S = 600.0                      # 표시 구간 [s] — 10분 기준 크롭

SIM_C, PEAK_C, TXT_C, GRID_C = "#0072B2", "#d62728", "#222222", "#e2e2e2"


def _parse(times, vals):
    F, sec = [], []
    for v, tm in zip(vals, times):
        mv = re.search(r"(-?\d+)\s*N", str(v)); mt = re.search(r"(\d+):(\d+):(\d+)", str(tm))
        if not (mv and mt):
            continue
        F.append(float(mv.group(1)))
        h, m, s = map(int, mt.groups()); sec.append(h * 3600 + m * 60 + s)
    return np.array(F), np.array(sec, dtype=float)


def load_txt(path):
    F, times = [], []
    with open(path, encoding="cp949", errors="replace") as f:
        for line in f:
            p = [x.strip() for x in line.split("|")]
            if len(p) < 4 or not p[0].isdigit():
                continue
            F.append(p[2]); times.append(p[1])
    return _parse(times, F)


def load_xls(path):
    import pandas as pd
    df = pd.read_excel(path, header=None)
    times, vals = [], []
    for _, row in df.iterrows():
        if not re.match(r"^\d+$", str(row[0]).strip()):
            continue
        times.append(row[1]); vals.append(row[2])
    F, sec = _parse(times, vals)
    return F[::-1], sec[::-1]           # .xls 는 NO. 내림차순(역시간) → 뒤집기


def to_time(F, sec):
    for i in range(1, len(sec)):
        while sec[i] < sec[i - 1] - 1:  # 12h/자정 롤오버
            sec[i] += 12 * 3600
    dur = sec[-1] - sec[0]; fs = len(F) / max(dur, 1.0)
    return np.arange(len(F)) / fs, fs, dur


def main():
    os.makedirs(OUT, exist_ok=True)
    plt.rcParams.update({"font.size": 12})
    for a in ANGLES:
        txt = os.path.join(DATA, f"{a}deg.txt"); xls = glob.glob(os.path.join(DATA, f"{a}deg*.xls"))
        if os.path.exists(txt):
            F, sec = load_txt(txt)
        elif xls:
            F, sec = load_xls(xls[0])
        else:
            print(f"[skip] {a}° 데이터 없음"); continue
        if len(F) < 2:
            print(f"[skip] {a}° 파싱 실패"); continue
        t, fs, dur = to_time(F, sec)
        m = t <= CROP_S                              # 10분 크롭 (peak도 크롭 구간 내에서)
        t, F = t[m], F[m]; dur = min(dur, CROP_S)
        pk = F.max(); ipk = int(np.argmax(F))

        fig, ax = plt.subplots(figsize=(11, 4.2), dpi=150)
        fig.patch.set_facecolor("white"); ax.set_facecolor("white")
        ax.plot(t, F, color=SIM_C, lw=0.8)
        ax.plot(t[ipk], pk, "o", color=PEAK_C, ms=8, zorder=5)
        ax.annotate(f"peak  {pk:.0f} N", (t[ipk], pk), xytext=(12, -2),
                    textcoords="offset points", color=PEAK_C, fontsize=13, fontweight="bold")
        for s in ["top", "right"]:
            ax.spines[s].set_visible(False)
        for sp in ["left", "bottom"]:
            ax.spines[sp].set_color("#888")
        ax.tick_params(colors=TXT_C, labelsize=12); ax.grid(True, color=GRID_C, lw=0.9)
        ax.set_ylim(bottom=0); ax.set_xlim(0, CROP_S)
        ax.set_xlabel("Time  [s]", color=TXT_C, fontsize=14)
        ax.set_ylabel("Crushing force  [N]", color=TXT_C, fontsize=14)
        ax.set_title(f"Measured crushing force over time  —  contact angle {a}°  (real ForceGage)",
                     color="#111111", fontweight="bold", fontsize=14.5, loc="left", pad=10)
        ax.text(0.99, 0.045, f"{fs:.1f} Hz  ·  {dur:.0f} s  ·  {len(F)} samples",
                transform=ax.transAxes, color="#666", fontsize=10, ha="right")
        fig.tight_layout()
        out = os.path.join(OUT, f"{a}deg_reaction_profile.png")
        fig.savefig(out, facecolor="white", bbox_inches="tight"); plt.close(fig)
        print(f"[saved] {a:3d}°  peak={pk:.0f} N  ({fs:.1f} Hz, {dur:.0f} s)  -> {out}")


if __name__ == "__main__":
    main()

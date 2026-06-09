"""
convert_xls_to_csv.py  —  Data_90.xls → CSV 변환 + 플롯 생성

사용법:
    python convert_xls_to_csv.py [--input Data_90.xls] [--out_dir .]
"""

import os
import re
import argparse
import warnings

import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

warnings.filterwarnings("ignore")   # OLE2 경고 무시

_HERE = os.path.dirname(os.path.abspath(__file__))


# ── 파싱 ─────────────────────────────────────────────────────────────────────
def parse_xls(path: str) -> tuple[pd.DataFrame, dict]:
    xl   = pd.ExcelFile(path)
    raw  = xl.parse(xl.sheet_names[0], header=None)

    # row 0: 컬럼 헤더, row 1: 통계 요약, row 2~: 데이터
    stats_row = raw.iloc[1].astype(str).str.cat(sep=" ")
    stats = {}
    for key, pat in [("max_N", r"Maximum:([\d.]+)N"),
                     ("min_N", r"Minimum:([\d.]+)N"),
                     ("avg_N", r"Average:([\d.]+)N")]:
        m = re.search(pat, stats_row)
        stats[key] = float(m.group(1)) if m else None

    data = raw.iloc[2:].copy()
    data.columns = ["no", "time_raw", "value_raw", "pressure_raw"]
    data["no"]           = pd.to_numeric(data["no"], errors="coerce")
    data["force_N"]      = data["value_raw"].astype(str).str.extract(r"([\d.]+)").astype(float)
    data["pressure_MPa"] = data["pressure_raw"].astype(str).str.extract(r"([\d.]+)").astype(float)

    # NO 기준 오름차순 정렬 (원본은 역순)
    data = data.dropna(subset=["no"]).sort_values("no").reset_index(drop=True)

    # 샘플 인덱스 (NO 기준 0-base)
    data["sample_idx"] = data["no"] - data["no"].min()

    return data[["no", "sample_idx", "force_N", "pressure_MPa"]], stats


# ── CSV 저장 ──────────────────────────────────────────────────────────────────
def save_csv(data: pd.DataFrame, stats: dict, out_path: str):
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(f"# source: Data_90.xls\n")
        f.write(f"# max_force_N={stats['max_N']}  min_force_N={stats['min_N']}  avg_force_N={stats['avg_N']}\n")
    data.to_csv(out_path, mode="a", index=False)
    print(f"  CSV 저장: {out_path}")


# ── 플롯 ──────────────────────────────────────────────────────────────────────
def make_plots(data: pd.DataFrame, stats: dict, out_dir: str, stem: str):
    force  = data["force_N"].values
    idx    = data["sample_idx"].values
    n      = len(force)

    # 비영(non-zero) 구간만
    nonzero_mask  = force > 0
    peak_idx      = int(np.argmax(force))
    peak_force    = float(force[peak_idx])

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle(
        f"Hardness Test — Data_90  |  N={n} samples\n"
        f"Max={stats['max_N']} N   Min={stats['min_N']} N   Avg={stats['avg_N']} N",
        fontsize=12, fontweight="bold"
    )

    # ── ① 전체 Force 시계열 ───────────────────────────────────────────────────
    ax = axes[0]
    ax.plot(idx, force, color="steelblue", lw=1.0, label="Force [N]")
    ax.axhline(stats["avg_N"], color="orange", ls="--", lw=1.2,
               label=f"Avg {stats['avg_N']} N")
    ax.axhline(stats["max_N"], color="red",    ls=":",  lw=1.0,
               label=f"Max {stats['max_N']} N")
    ax.scatter([idx[peak_idx]], [peak_force], color="red", zorder=5, s=40,
               label=f"Peak @ idx={idx[peak_idx]:.0f}")
    ax.set_ylabel("Force [N]")
    ax.set_xlabel("Sample index (NO)")
    ax.set_title("① Full force time-series")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(idx[0], idx[-1])

    # ── ② 비영 구간 확대 ─────────────────────────────────────────────────────
    ax = axes[1]
    if nonzero_mask.any():
        nz_idx   = idx[nonzero_mask]
        nz_force = force[nonzero_mask]
        ax.plot(nz_idx, nz_force, color="darkorange", lw=1.2, marker=".", ms=3,
                label="Force > 0")
        ax.fill_between(nz_idx, 0, nz_force, alpha=0.15, color="darkorange")
        ax.axhline(stats["avg_N"], color="gray", ls="--", lw=1.0,
                   label=f"Avg {stats['avg_N']} N")
        # 피크 하강 경사 (최댓값 이후)
        peak_local = int(np.argmax(nz_force))
        ax.annotate(
            f"Peak\n{peak_force:.0f} N",
            xy=(nz_idx[peak_local], nz_force[peak_local]),
            xytext=(nz_idx[peak_local] + max(1, len(nz_idx)//10), nz_force[peak_local] * 0.85),
            arrowprops=dict(arrowstyle="->", color="red"),
            fontsize=8, color="red"
        )
    ax.set_ylabel("Force [N]")
    ax.set_xlabel("Sample index (NO)")
    ax.set_title("② Non-zero region (contact period)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── ③ 히스토그램 (전체 / 비영 분리) ─────────────────────────────────────
    ax = axes[2]
    bins_all = np.arange(0, force.max() + 10, 10)
    ax.hist(force[nonzero_mask], bins=bins_all, color="steelblue", alpha=0.7,
            label=f"Force > 0  (n={nonzero_mask.sum()})", edgecolor="white", lw=0.5)
    ax.axvline(stats["avg_N"], color="orange", ls="--", lw=1.5,
               label=f"Avg {stats['avg_N']} N")
    ax.axvline(stats["max_N"], color="red",    ls=":",  lw=1.2,
               label=f"Max {stats['max_N']} N")
    ax.set_xlabel("Force [N]")
    ax.set_ylabel("Count")
    ax.set_title("③ Force distribution (non-zero samples)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, axis="y")
    ax.xaxis.set_minor_locator(ticker.MultipleLocator(10))

    fig.tight_layout()
    out_png = os.path.join(out_dir, f"{stem}_plot.png")
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Plot 저장: {out_png}")
    return out_png


# ── main ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Data_90.xls → CSV + plot")
    parser.add_argument("--input",   default=os.path.join(_HERE, "Data_90.xls"))
    parser.add_argument("--out_dir", default=_HERE)
    args = parser.parse_args()

    xls_path = os.path.abspath(args.input)
    out_dir  = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(xls_path))[0]

    print(f"\n[1/3] 파싱 중: {xls_path}")
    data, stats = parse_xls(xls_path)
    print(f"  rows={len(data)}  max={stats['max_N']} N  min={stats['min_N']} N  avg={stats['avg_N']} N")

    print(f"\n[2/3] CSV 저장 중 ...")
    csv_path = os.path.join(out_dir, f"{stem}.csv")
    save_csv(data, stats, csv_path)

    print(f"\n[3/3] 플롯 생성 중 ...")
    make_plots(data, stats, out_dir, stem)

    print(f"\n완료.")


if __name__ == "__main__":
    main()

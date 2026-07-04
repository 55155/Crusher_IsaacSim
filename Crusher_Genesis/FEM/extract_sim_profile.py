"""Crusher_8env 의 MuJoCo 시뮬에서 θ=60° 벽 반력 시계열 F(t) 을 추출 → CSV.
(Crusher_8env.py 는 요약 peak 만 저장하므로, FEM 구동용 시계열을 따로 뽑는다.)"""
import os, sys, csv, math
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
CRU = os.path.dirname(_HERE)                       # Crusher_Genesis
sys.path.insert(0, CRU)
import Crusher_8env as C8

THETA = int(sys.argv[1]) if len(sys.argv) > 1 else 60
OUT = os.path.join(CRU, "Sim_result", "sim8env", f"{THETA}deg_sim_Ft.csv")

print(f"[map] slider_Y(θ) 매핑 ...")
ymap = C8.slider_y_map()
wall_face = ymap[min(ymap, key=lambda k: abs(k + (THETA - C8.PHASE_OFFSET_DEG)))] + C8.PLATE_C
print(f"[run] run_angle({THETA}) ...")
t, f, cdeg, peaks = C8.run_angle(THETA, wall_face)
print(f"[done] {len(t)} samples, contact~{abs(cdeg) if cdeg else THETA:.1f}°, "
      f"mean_peak={peaks.mean() if len(peaks) else 0:.1f} N, strikes={len(peaks)}")

with open(OUT, "w", newline="") as fh:
    wt = csv.writer(fh); wt.writerow(["t_s", "F_N"])
    wt.writerows(zip(t.tolist(), f.tolist()))
print(f"[saved] {OUT}")

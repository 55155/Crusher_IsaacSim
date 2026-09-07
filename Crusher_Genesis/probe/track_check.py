import sys, numpy as np, imageio.v3 as iio
v = sys.argv[1]
rows=[]
for i, fr in enumerate(iio.imiter(v, plugin="pyav")):
    if i % 20: continue
    f = fr.astype(np.int16)
    R,G,B = f[...,0], f[...,1], f[...,2]
    m = (R-G > 18) & (R-B > 18) & (R > 90)          # 붉은 실링 스트라이프
    n = int(m.sum())
    if n < 60:
        rows.append((i, n, np.nan, np.nan)); continue
    ys, xs = np.nonzero(m)
    rows.append((i, n, xs.mean(), ys.mean()))
H, W = fr.shape[:2]
print(f"frames={i+1} res={W}x{H} center=({W/2:.0f},{H/2:.0f})")
ok = [r for r in rows if not np.isnan(r[2])]
print(f"샘플 {len(rows)} 중 실링 검출 {len(ok)} ({100*len(ok)/len(rows):.0f}%)")
if ok:
    dx = np.array([r[2]-W/2 for r in ok]); dy = np.array([r[3]-H/2 for r in ok])
    off = np.hypot(dx, dy)
    print(f"중심오차 px: 중앙값={np.median(off):.0f}  90%={np.percentile(off,90):.0f}  최대={off.max():.0f}")
    print(f"  화면 반폭 대비 중앙값 {100*np.median(off)/(W/2):.0f}%")
for r in rows[::5]:
    print(f"  n={r[0]:5d} px={r[1]:6d} cx={r[2]:7.1f} cy={r[3]:7.1f}" if not np.isnan(r[2])
          else f"  n={r[0]:5d} px={r[1]:6d}  --놓침--")

import re, sys, numpy as np, collections
src = open(sys.argv[1], encoding="utf-8", errors="replace").read()
src = re.sub(r"\s*\n\s*", " ", src)                       # 개체가 줄바꿈으로 쪼개져 있다
ent = dict(re.findall(r"#(\d+)\s*=\s*([A-Z_0-9]+\s*\([^;]*)\;", src))
pts = {}
for k, v in ent.items():
    m = re.match(r"CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(([^)]*)\)", v)
    if m:
        c = [float(x) for x in m.group(1).split(",")]
        if len(c) == 3: pts[k] = np.array(c)
P = np.array(list(pts.values()))
print(f"CARTESIAN_POINT {len(P)}개")
print("  bbox min", P.min(0).round(3), "max", P.max(0).round(3))
print("  extents ", (P.max(0)-P.min(0)).round(3))

# AXIS2_PLACEMENT_3D -> 원점 포인트
ax = {}
for k, v in ent.items():
    m = re.match(r"AXIS2_PLACEMENT_3D\s*\(\s*'[^']*'\s*,\s*#(\d+)\s*,\s*#(\d+)\s*,\s*#(\d+)", v)
    if m: ax[k] = (m.group(1), m.group(2), m.group(3))
dirs = {}
for k, v in ent.items():
    m = re.match(r"DIRECTION\s*\(\s*'[^']*'\s*,\s*\(([^)]*)\)", v)
    if m: dirs[k] = np.array([float(x) for x in m.group(1).split(",")])

cyl = []
for k, v in ent.items():
    m = re.match(r"CYLINDRICAL_SURFACE\s*\(\s*'[^']*'\s*,\s*#(\d+)\s*,\s*([0-9.E+-]+)", v)
    if m and m.group(1) in ax:
        o, d, _ = ax[m.group(1)]
        if o in pts: cyl.append((round(float(m.group(2)), 4), pts[o], dirs.get(d)))
print(f"\nCYLINDRICAL_SURFACE {len(cyl)}개")
for r, n in sorted(collections.Counter(c[0] for c in cyl).items()):
    print(f"  R={r:8.4f}mm (D={2*r:7.3f})  x{n}")

C = np.array([c[1] for c in cyl])
ux, uy = np.unique(C[:,0].round(3)), np.unique(C[:,1].round(3))
print(f"\n구멍 중심 고유 X {len(ux)}개: {ux[:6]} ... {ux[-3:]}")
print(f"구멍 중심 고유 Y {len(uy)}개: {uy[:6]} ... {uy[-3:]}")
print(f"X 피치 {np.unique(np.diff(ux).round(3))}  Y 피치 {np.unique(np.diff(uy).round(3))}")
print(f"X 여백 좌 {ux.min()-(-750):.1f} 우 {750-ux.max():.1f} / Y 여백 하 {uy.min()-0:.1f} 상 {800-uy.max():.1f}")
print(f"격자 {len(ux)}x{len(uy)} = {len(ux)*len(uy)} (실제 {len(C)})")
print(f"구멍 z 범위 {C[:,2].min():.1f} ~ {C[:,2].max():.1f}")

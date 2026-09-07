"""무게별 모멘트 — M = (CoM - 파지점) x W, W=(0,0,-mg).
   외적 성질상 M 은 **수평 오프셋에만** 비례한다(연직 성분은 기여 0, Mz=0)."""
import sys, glob, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt, matplotlib.font_manager as fm
for _f in ("Malgun Gothic","Gulim","NanumGothic","DejaVu Sans"):
    if any(_f in f.name for f in fm.fontManager.ttflist):
        matplotlib.rcParams["font.family"]=_f; break
matplotlib.rcParams["axes.unicode_minus"]=False
files = sorted(glob.glob(sys.argv[1]), key=lambda x:int(x.split("_N")[1].split("_")[0]))
fig = plt.figure(figsize=(16,5))
ax1 = fig.add_subplot(131, projection="3d"); ax2 = fig.add_subplot(132); ax3 = fig.add_subplot(133)
cm = plt.get_cmap("viridis"); rows=[]
for i,f in enumerate(files):
    d=np.load(f); h=d["hist"]; g=d["grip"]
    t=h[:,0]; com=h[:,1:4]; M=h[:,4:7]; n=h[:,7]
    mass=float(d["m_grain"])*int(d["n_grains"])
    c=cm(i/max(len(files)-1,1)); lab=f"{mass*1e3:.2f} g"
    r=(com-g); rh=np.linalg.norm(r[:,:2],axis=1)*1e3          # 수평 오프셋 mm
    Mmag=np.linalg.norm(M,axis=1)*1e3                          # mN*m
    ax1.plot((com[:,0]-g[0])*1e3,(com[:,1]-g[1])*1e3,com[:,2]*1e3,color=c,lw=1.6,label=lab)
    ax1.scatter((com[-1,0]-g[0])*1e3,(com[-1,1]-g[1])*1e3,com[-1,2]*1e3,color=c,s=50,zorder=5)
    ax2.plot(t,rh,color=c,lw=1.8,label=lab)
    ax3.plot(t,Mmag,color=c,lw=1.8,label=lab)
    rows.append((mass*1e3,int(n[-1]),rh[-1],Mmag[-1],mass*9.81*1e3))
ax1.scatter(0,0,float(np.load(files[0])["grip"][2])*1e3,color="crimson",s=110,marker="X",label="파지점")
ax1.set_title("(a) 파지점 기준 파우더 무게중심"); ax1.set_xlabel("Δx [mm]"); ax1.set_ylabel("Δy [mm]"); ax1.set_zlabel("z [mm]")
ax1.legend(fontsize=8,loc="upper left")
ax2.set_title("(b) 수평 오프셋 |Δr_xy|  ← 모멘트를 만드는 유일한 성분")
ax2.set_xlabel("t [s]"); ax2.set_ylabel("수평 오프셋 [mm]"); ax2.grid(alpha=.3); ax2.legend(fontsize=8)
ax3.set_title("(c) 파지점 모멘트 |M|"); ax3.set_xlabel("t [s]"); ax3.set_ylabel("|M| [mN·m]")
ax3.grid(alpha=.3); ax3.legend(fontsize=8)
plt.tight_layout(); plt.savefig(sys.argv[2],dpi=140); print("[plot]",sys.argv[2])
print(f"{'질량[g]':>7} {'알':>6} {'수평오프셋[mm]':>14} {'|M|[mN·m]':>11} {'무게[mN]':>9} {'M/W[mm]':>9}")
for m,n,rh,M,W in rows: print(f"{m:7.2f} {n:6d} {rh:14.3f} {M:11.5f} {W:9.2f} {M/W:9.3f}")

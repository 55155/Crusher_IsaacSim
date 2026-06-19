"""
medicine_envelope_pbd_box.py
열린 약봉투(open-top Medicine Envelope) + 낙하하는 작은 Box 엔티티 실험.

목적:
  상단이 열린 봉투(5 panel) 안으로 Genesis 내장 Box 엔티티(rigid)를 떨어뜨려,
  PBD cloth 파티클의 "응집력(cohesion)"이 크게 변하는지 헤드리스로 측정한다.

비교:
  (A) baseline   : 박스 없음
  (B) with_box   : 작은 rigid Box 낙하

응집력 지표:
  - σ (positional spread, std)         : 작을수록 응집
  - mean nearest-neighbor distance     : 작을수록 응집
  - radius of gyration Rg              : 작을수록 응집

medicine_envelope.py 파라미터:
  W=8cm(X)  H=12cm(Y→World-Z)  D=1cm(Z→World-Y)

사용:
  python medicine_envelope_pbd_box.py
"""

import os, time
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ─── 봉투 파라미터 (medicine_envelope.py 동일) ───────────────
W = 0.08    # 폭   [m]
H = 0.12    # 높이 [m]
D = 0.01    # 깊이 [m]
NW, NH, ND = 8, 12, 3      # 메시 분할 (삼각형 해상도; particle 수는 particle_size가 결정)

# ─── 시뮬 ────────────────────────────────────────────────
DT        = 5e-4
SUBSTEPS  = 10
SIM_TIME  = 1.5
LOG_EVERY = 20
PARTICLE_SIZE = 2e-3

# ─── 낙하 박스 (열린 상단 통과 가능하도록 작게) ───────────────
#   열린 상단 opening ≈ 8cm(X) × 1cm(Y).  박스 Y치수 < 1cm 필요.
BOX_SIZE = (0.015, 0.006, 0.015)   # X,Y,Z [m]  = 1.5 × 0.6 × 1.5 cm
BOX_RHO  = 400.0                   # [kg/m^3]
BOX_DROP_Z = 0.235                 # 낙하 시작 높이 [m] (봉투 상단 ~0.17 위)

# ─── 출력 ────────────────────────────────────────────────
_DIR    = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "Sim_result")
os.makedirs(OUT_DIR, exist_ok=True)
STL_PATH  = os.path.join(OUT_DIR, "medicine_envelope_open.stl")
PLOT_PATH = os.path.join(OUT_DIR, "medicine_envelope_box_cohesion.png")
CSV_PATH  = os.path.join(OUT_DIR, "medicine_envelope_box_cohesion.csv")


# ══════════════════════════════════════════════════════════
#  메시 (열린 상단 5-panel)
# ══════════════════════════════════════════════════════════
def _panel(fn, nu, nv):
    tris = []
    for i in range(nu):
        for j in range(nv):
            u0, u1 = i/nu, (i+1)/nu
            v0, v1 = j/nv, (j+1)/nv
            p00 = fn(u0, v0); p10 = fn(u1, v0)
            p11 = fn(u1, v1); p01 = fn(u0, v1)
            tris.append([p00, p10, p11]); tris.append([p00, p11, p01])
    return tris


def create_open_envelope():
    """상단 열린 5-panel 봉투 → vertex merge → origin 중심 → STL."""
    import trimesh as _tm
    tris = []
    tris += _panel(lambda u, v: np.array([u*W, v*H, 0.0]), NW, NH)   # Front
    tris += _panel(lambda u, v: np.array([u*W, v*H, D  ]), NW, NH)   # Back
    tris += _panel(lambda u, v: np.array([u*W, 0.0, v*D]), NW, ND)   # Bottom
    tris += _panel(lambda u, v: np.array([0.0, u*H, v*D]), NH, ND)   # Left
    tris += _panel(lambda u, v: np.array([W,   u*H, v*D]), NH, ND)   # Right
    verts = np.array([p for t in tris for p in t])
    faces = np.arange(len(verts)).reshape(-1, 3)
    m = _tm.Trimesh(vertices=verts, faces=faces, process=False)
    m.merge_vertices(digits_vertex=7)
    m.vertices -= m.bounding_box.centroid
    m.export(STL_PATH)
    print(f"[mesh] open-top  tris={len(m.faces)}  verts={len(m.vertices)}  "
          f"watertight={m.is_watertight}")
    return m


# ══════════════════════════════════════════════════════════
#  유틸
# ══════════════════════════════════════════════════════════
def _pos(entity):
    p = entity.get_particles_pos()
    if hasattr(p, "cpu"):
        p = p.cpu().numpy()
    p = np.asarray(p, dtype=np.float64)
    if p.ndim == 3:
        p = p[0]
    return p


def cohesion_metrics(pos):
    """응집력 지표 (σ, mean-NN, Rg) 계산."""
    valid = pos[~np.isnan(pos).any(axis=1)]
    com   = valid.mean(axis=0)
    sigma = valid.std(axis=0)                       # (3,)
    rg    = np.sqrt(np.mean(np.sum((valid - com)**2, axis=1)))
    # mean nearest-neighbor distance (자기 자신 제외 → k=2)
    tree  = cKDTree(valid)
    dd, _ = tree.query(valid, k=2)
    nn    = dd[:, 1].mean()
    return com, sigma, rg, nn


def find_top_corners(pos0):
    z, x, y = pos0[:, 2], pos0[:, 0], pos0[:, 1]
    top = np.where(z >= np.quantile(z, 0.95))[0]
    c = [top[np.argmin(x[top]+y[top])], top[np.argmax(x[top]-y[top])],
         top[np.argmin(-x[top]+y[top])], top[np.argmax(x[top]+y[top])]]
    return list({int(i) for i in c})


# ══════════════════════════════════════════════════════════
#  단일 조건 시뮬
# ══════════════════════════════════════════════════════════
def run_case(gs, with_box):
    tag = "with_box" if with_box else "baseline"
    print(f"\n{'='*56}\n  CASE: {tag}\n{'='*56}")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS,
                                          gravity=(0, 0, -9.81)),
        pbd_options=gs.options.PBDOptions(
            max_stretch_solver_iterations=25,
            max_bending_solver_iterations=8,
            max_volume_solver_iterations=3,
            max_density_solver_iterations=1,
            max_viscosity_solver_iterations=1,
            particle_size=PARTICLE_SIZE,
        ),
        coupler_options=gs.options.LegacyCouplerOptions(rigid_pbd=True),
        show_viewer=False,
    )
    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid())

    pos_z = 0.05 + H/2
    bag = scene.add_entity(
        material=gs.materials.PBD.Cloth(),
        morph=gs.morphs.Mesh(file=STL_PATH, scale=1.0,
                             pos=(0, 0, pos_z), euler=(90, 0, 0)),
    )

    box = None
    if with_box:
        box = scene.add_entity(
            material=gs.materials.Rigid(rho=BOX_RHO),
            morph=gs.morphs.Box(size=BOX_SIZE, pos=(0.0, 0.0, BOX_DROP_Z),
                                fixed=False),
        )

    t0 = time.time()
    scene.build(n_envs=0)
    print(f"[build] OK ({time.time()-t0:.1f}s)")

    pos0 = _pos(bag)
    N = pos0.shape[0]
    fix_idx = find_top_corners(pos0)
    bag.fix_particles(particles_idx_local=fix_idx)
    print(f"[N]={N}  fixed top corners={len(fix_idx)}")

    n_steps = int(SIM_TIME / DT)
    log = dict(t=[], sx=[], sy=[], sz=[], rg=[], nn=[], box_z=[])
    snapshots = {0.0: pos0.copy()}
    snap_times = [0.0, 0.5, 1.0, 1.5]

    print(f"  {'t[s]':>6} | {'σZ[cm]':>7} | {'Rg[cm]':>7} | {'NN[mm]':>7} | {'box_z[cm]':>9}")
    print("  " + "-"*52)
    ts = time.time()
    for k in range(n_steps):
        scene.step()
        t = (k+1)*DT
        for st in snap_times[1:]:
            if st not in snapshots and abs(t-st) < DT*0.5:
                snapshots[st] = _pos(bag).copy()
        if (k+1) % LOG_EVERY == 0:
            p = _pos(bag)
            com, sigma, rg, nn = cohesion_metrics(p)
            bz = np.nan
            if box is not None:
                bp = box.get_pos()
                if hasattr(bp, "cpu"): bp = bp.cpu().numpy()
                bp = np.asarray(bp).reshape(-1)
                bz = float(bp[2])
            log["t"].append(t); log["sx"].append(sigma[0]); log["sy"].append(sigma[1])
            log["sz"].append(sigma[2]); log["rg"].append(rg); log["nn"].append(nn)
            log["box_z"].append(bz)
            if (k+1) % (LOG_EVERY*10) == 0:
                print(f"  {t:6.3f} | {sigma[2]*100:7.3f} | {rg*100:7.3f} | "
                      f"{nn*1000:7.3f} | {bz*100:9.3f}")
    print(f"[sim] wall={time.time()-ts:.1f}s")

    for k in log: log[k] = np.array(log[k])
    return dict(tag=tag, N=N, log=log, snapshots=snapshots, snap_times=snap_times)


# ══════════════════════════════════════════════════════════
#  플롯
# ══════════════════════════════════════════════════════════
def make_plot(base, boxc):
    fig = plt.figure(figsize=(17, 11))
    fig.suptitle("Medicine Envelope (open-top) — Particle Cohesion: baseline vs falling Box\n"
                 f"W={W*100:.0f} H={H*100:.0f} D={D*100:.1f} cm   "
                 f"N≈{base['N']}   box={BOX_SIZE[0]*100:.1f}×{BOX_SIZE[1]*100:.1f}×{BOX_SIZE[2]*100:.1f} cm",
                 fontsize=13, fontweight="bold")

    cb, cx = "tab:blue", "tab:red"

    # Row1: 응집 지표 시계열
    def ts_ax(pos, key, ylabel, title, scale):
        ax = fig.add_subplot(3, 4, pos)
        ax.plot(base["log"]["t"], base["log"][key]*scale, color=cb, lw=1.6, label="baseline")
        ax.plot(boxc["log"]["t"], boxc["log"][key]*scale, color=cx, lw=1.6, label="with_box")
        ax.set_xlabel("t [s]"); ax.set_ylabel(ylabel); ax.set_title(title)
        ax.legend(fontsize=8); ax.grid(alpha=0.3)
        return ax
    ts_ax(1, "nn", "mean NN [mm]", "Cohesion: mean nearest-neighbor dist", 1000)
    ts_ax(2, "rg", "Rg [cm]", "Radius of gyration", 100)
    ts_ax(3, "sz", "σ Z [cm]", "Spread σ (vertical)", 100)
    ax_b = fig.add_subplot(3, 4, 4)
    ax_b.plot(boxc["log"]["t"], boxc["log"]["box_z"]*100, color=cx, lw=1.6)
    ax_b.axhline(17, color="gray", ls="--", lw=1, label="bag top ~17cm")
    ax_b.axhline(5,  color="black", ls=":", lw=1, label="bag bottom ~5cm")
    ax_b.set_xlabel("t [s]"); ax_b.set_ylabel("box Z [cm]")
    ax_b.set_title("Box center height (drop)"); ax_b.legend(fontsize=7); ax_b.grid(alpha=0.3)

    # Row2/3: 스냅샷 YZ 투영 (baseline=2,3 / box=4,5 위치)
    def scatter_case(case, base_idx, color, title):
        for i, st in enumerate(case["snap_times"]):
            if st not in case["snapshots"]: continue
            ax = fig.add_subplot(3, 4, base_idx + i)
            p = case["snapshots"][st]
            v = p[~np.isnan(p).any(axis=1)]
            ax.scatter(v[:,0]*100, v[:,2]*100, s=2, alpha=0.4, color=color)
            ax.set_xlim(-6,6); ax.set_ylim(2,20)
            ax.set_xlabel("X[cm]"); ax.set_ylabel("Z[cm]")
            ax.set_title(f"{title} t={st:.1f}s"); ax.grid(alpha=0.25)
    scatter_case(base, 5, cb, "baseline")
    scatter_case(boxc, 9, cx, "with_box")

    fig.tight_layout()
    fig.savefig(PLOT_PATH, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"[saved] {PLOT_PATH}")


# ══════════════════════════════════════════════════════════
def main():
    print("="*60)
    print(" Open-top Medicine Envelope + falling Box — cohesion test")
    print(f"   box size={BOX_SIZE} m  rho={BOX_RHO}  drop_z={BOX_DROP_Z} m")
    print("="*60)

    create_open_envelope()

    import genesis as gs
    gs.init(backend=gs.metal, logging_level="warning")

    base = run_case(gs, with_box=False)
    boxc = run_case(gs, with_box=True)

    make_plot(base, boxc)

    # 요약 비교 (마지막 시점)
    def last(c, k): return c["log"][k][-1]
    print("\n" + "="*60)
    print(" COHESION SUMMARY (t = last)")
    print("="*60)
    print(f"  {'metric':<18}{'baseline':>12}{'with_box':>12}{'Δ%':>10}")
    for key, sc, unit in [("nn",1000,"mm"), ("rg",100,"cm"),
                          ("sz",100,"cm"), ("sx",100,"cm"), ("sy",100,"cm")]:
        b, x = last(base,key)*sc, last(boxc,key)*sc
        dpct = (x-b)/b*100 if b != 0 else 0
        print(f"  {key+' ['+unit+']':<18}{b:>12.4f}{x:>12.4f}{dpct:>+9.2f}%")

    # CSV
    with open(CSV_PATH, "w") as fh:
        fh.write("case,t_s,sigma_x_m,sigma_y_m,sigma_z_m,Rg_m,meanNN_m,box_z_m\n")
        for c in (base, boxc):
            L = c["log"]
            for i in range(len(L["t"])):
                fh.write(f"{c['tag']},{L['t'][i]:.4f},{L['sx'][i]:.6f},"
                         f"{L['sy'][i]:.6f},{L['sz'][i]:.6f},{L['rg'][i]:.6f},"
                         f"{L['nn'][i]:.6f},{L['box_z'][i]:.6f}\n")
    print(f"[saved] {CSV_PATH}")
    print("\n완료.")


if __name__ == "__main__":
    main()

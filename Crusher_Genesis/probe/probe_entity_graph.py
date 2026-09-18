"""probe_entity_graph.py — scene entities as photos, wired by what they do.

2026-09-14. Revisions after user feedback:
  1) 초판: 색 4개 + 설정값 나열 + 우측 IPC 설명 박스  -> 너무 많다
  2) 2판: 색 2개, 라벨 짧게, 봉투 중심 고리 배치
  3) 3판(현재):
     - **"IPC 접촉"을 간선 라벨에서 뺐다.** 봉투와 닿는 것은 **전부** IPC 커플러를
       지나므로 그걸 간선마다 적으면 정보가 0 이고 오히려 헷갈린다. 간선은 그
       접촉이 공정상 **무엇을 하는가**(Friction grasp / Crushing / Jaw clamping)만
       적고, IPC 라는 사실은 하단에 한 줄로 둔다.
     - **라벨을 영어로.** 한글로 옮기면 "압착·타격" 처럼 억지 합성어가 된다.
     - **노드마다 재료(표현)를 명시.** FEM.Cloth / FEM.Elastic / IPC Particle /
       Rigid — 이게 이 그림에서 가장 중요한 정보다.
     - 흡착 노드 그림을 파우더 있는 mouth 스냅샷에서 **파우더 없는 런**
       (RESULT_suck_video)의 프레임으로 교체.

Sources: rendered frames (rigid assemblies), baked STL (tablet), measured
positions from a crush npz (powder). Wiring read from `scene.add_entity` in
`full_workflow.py`.

Usage:  python probe_entity_graph.py
"""
import os
import glob
import subprocess

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.patches import FancyArrowPatch

matplotlib.rcParams["font.family"] = "DejaVu Sans"
matplotlib.rcParams["axes.unicode_minus"] = False

_HERE = os.path.dirname(os.path.abspath(__file__))
_WF = os.path.join(os.path.dirname(_HERE), "Crusher_M0609_RG2_Tablet_Samplebag")
_FR = os.path.join(_HERE, "_entity_frames")
FF = (r"C:\Users\simuser\anaconda3\envs\isaacsim\Lib\site-packages"
      r"\imageio_ffmpeg\binaries\ffmpeg-win-x86_64-v7.1.exe")
# 파우더 없는 런들. 흡착 노드는 suck_video 쪽에서 뽑는다.
VID_MAIN = os.path.join(_WF, "RESULT", "attach_20260902", "E_attach_crush60_%s.mp4")
VID_SUCK = os.path.join(_WF, "RESULT_suck_video", "grip-28mm",
                        "full_workflow_grip-28mm_yoff+0.0mm_20260902_220231_bagcam.mp4")

BLUE = "#0072B2"      # two-way: reaction returns to the body
GREY = "#9aa3ab"      # support only (ipc_only)

# ── 자산 출처 배지 (2026-09-14, 4판) ────────────────────────────────────────
# "정제는 스크립트로 만든다"를 아이콘으로 어떻게 적을까가 출발이었는데, 스크립트만
# 특별 취급하면 축이 하나 늘 뿐이다. **모든 노드에 '이 자산이 어디서 왔나'를
# 같은 자리에 같은 모양으로** 붙이면 .py 는 그 축의 한 값이 되고, 덤으로
# "판·봉투도 스크립트 산출물"이라는 사실이 같이 드러난다.
PROV = {
    "xml": ("CAD - MJCF", "#5b6b7a"),     # Fusion 360 -> MJCF
    "stl": ("CAD - mesh", "#7a6a5b"),     # Fusion 360 -> STL
    "py":  ("script", "#0b7a5b"),         # 코드가 형상을 만든다
    "prim": ("primitive", "#8a8a8a"),     # gs.morphs.Box / Plane
}


def _grab(video, tag, n):
    p = os.path.join(_FR, "%s_%d.png" % (tag, n))
    if not os.path.exists(p):
        subprocess.run([FF, "-y", "-i", video, "-vf", "select='eq(n\\,%d)'" % n,
                        "-vsync", "0", "-frames:v", "1", p],
                       check=True, capture_output=True)
    return mpimg.imread(p)


def _tablet_img():
    """Tablet: the baked capsule STL, shaded by face normal."""
    import trimesh as tm
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    m = tm.load(os.path.join(_WF, "RESULT", "_analytic_capsule_v2.stl"))
    v, f = m.vertices, m.faces
    fig = plt.figure(figsize=(2.4, 2.4), dpi=150)
    ax = fig.add_subplot(111, projection="3d")
    light = np.array([0.4, -0.7, 0.6]); light /= np.linalg.norm(light)
    sh = np.clip(m.face_normals @ light, 0.15, 1.0)
    cols = np.stack([0.62 + 0.34 * sh, 0.60 + 0.33 * sh, 0.88 - 0.06 * sh,
                     np.ones_like(sh)], axis=1)
    ax.add_collection3d(Poly3DCollection(v[f], facecolors=cols, edgecolor="none"))
    lo, hi = v.min(axis=0), v.max(axis=0)
    c, r = (lo + hi) / 2, (hi - lo).max() / 2 * 1.05
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1)); ax.view_init(elev=16, azim=-60)
    ax.axis("off")
    p = os.path.join(_FR, "_tablet.png")
    fig.savefig(p, facecolor="white", bbox_inches="tight", pad_inches=0)
    plt.close(fig)
    return mpimg.imread(p)


def _grain_img():
    """Powder: measured positions from a crush npz, at its bulkiest instant."""
    c = glob.glob(os.path.join(_WF, "RESULT_*", "grip*", "crush_*.npz"))
    if not c:
        return None
    d = np.load(max(c, key=os.path.getmtime), allow_pickle=True)
    P = d["fld_pos"].astype(float) * 1e3
    ext = P.max(axis=1) - P.min(axis=1)
    p = P[int(np.argmax(np.prod(ext, axis=1)))]
    p = p - p.mean(axis=0)
    fig = plt.figure(figsize=(2.4, 2.4), dpi=150)
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(p[:, 0], p[:, 1], p[:, 2], s=16, c="#c9a227", depthshade=True,
               edgecolors="#6b5510", linewidths=0.2)
    e = p.max(axis=0) - p.min(axis=0)
    asp = e / e.max(); asp[np.argmin(asp)] = max(asp.min(), 0.32)
    pad = 0.08 * e.max()
    ax.set_xlim(p[:, 0].min() - pad, p[:, 0].max() + pad)
    ax.set_ylim(p[:, 1].min() - pad, p[:, 1].max() + pad)
    ax.set_zlim(p[:, 2].min() - pad, p[:, 2].max() + pad)
    ax.set_box_aspect(tuple(asp)); ax.view_init(elev=20, azim=-62)
    ax.axis("off")
    out = os.path.join(_FR, "_grain.png")
    fig.savefig(out, facecolor="white", bbox_inches="tight", pad_inches=0)
    plt.close(fig)
    return mpimg.imread(out), int(d["n_grains"]), float(d["grain_radius"]) * 1e3


def main():
    os.makedirs(_FR, exist_ok=True)
    g = _grain_img()
    gtxt = ("IPC Particle x %d, R=%.1f mm" % (g[1], g[2])) if g else "IPC Particle"

    # (key, title, material line, image, crop, rect)
    N = [
        ("bag", "Sample bag", "FEM.Cloth  ·  64 x 90 x 6 mm, t = 1 mm", "stl",
         _grab(VID_MAIN % "bagcam", "bagcam", 140), (370, 175, 615, 585),
         (0.385, 0.335, 0.175, 0.300)),
        ("arm", "Robot M0609 + RG2", "Rigid (MJCF)  ·  finger links coupled", "xml",
         _grab(VID_MAIN % "overview", "overview", 700), (745, 155, 1080, 490),
         (0.085, 0.660, 0.165, 0.215)),
        ("tablet", "Tablet", "FEM.Elastic  ·  4 x 4 x 5 mm, 137 v / 288 tet", "py",
         _tablet_img(), None, (0.402, 0.730, 0.140, 0.170)),
        ("suck", "Suction unit V1", "Rigid (MJCF)  ·  two cups", "xml",
         _grab(VID_SUCK, "suck", 1567), (280, 130, 800, 470),
         (0.700, 0.660, 0.165, 0.215)),
        ("crusher", "Crusher", "Rigid (MJCF)  ·  wall + impact plate", "xml",
         _grab(VID_MAIN % "bagcam", "bagcam", 700), (240, 290, 775, 695),
         (0.790, 0.385, 0.175, 0.230)),
        ("recov", "Recovery unit", "Rigid (MJCF)  ·  jaw links coupled", "xml",
         _grab(VID_MAIN % "bagcam", "bagcam", 2780), (300, 250, 810, 690),
         (0.700, 0.140, 0.165, 0.215)),
        ("grain", "Powder", gtxt, "py", (g[0] if g else None), None,
         (0.400, 0.045, 0.145, 0.180)),
        ("shelf", "Shelf", "Rigid box  ·  support only", "prim",
         _grab(VID_MAIN % "bagcam", "bagcam", 20), (150, 340, 650, 600),
         (0.085, 0.140, 0.165, 0.190)),
        ("plate", "Base plate + ground", "Rigid mesh / plane  ·  support only", "py",
         _grab(VID_MAIN % "overview", "overview", 700), (525, 395, 1055, 715),
         (0.030, 0.395, 0.145, 0.185)),
    ]
    # 간선 라벨은 **그 접촉이 공정에서 하는 일**이다. "IPC" 는 안 적는다 —
    # 전부 IPC 라서 적으면 구별이 안 된다.
    E = [
        ("arm", "bag", BLUE, 3.2, "Friction grasp"),
        ("tablet", "bag", BLUE, 2.2, "Containment"),
        ("suck", "bag", BLUE, 2.6, "Mouth opening"),
        ("crusher", "bag", BLUE, 3.2, "Crushing"),
        ("recov", "bag", BLUE, 2.6, "Jaw clamping"),
        ("grain", "bag", BLUE, 3.2, "Containment\n+ collision"),
        ("shelf", "bag", GREY, 2.0, "Support"),
        ("plate", "shelf", GREY, 1.8, "Ground"),
    ]

    fig = plt.figure(figsize=(15.5, 10.5))
    fig.suptitle("Scene entities and their interactions "
                 "— everything couples through the sample bag",
                 fontsize=17, y=0.965)

    pos = {}
    for key, title, sub, prov, img, crop, rect in N:
        ax = fig.add_axes(rect)
        if img is not None:
            ax.imshow(img[crop[1]:crop[3], crop[0]:crop[2]] if crop else img)
        ax.set_xticks([]); ax.set_yticks([])
        big = key == "bag"
        for s in ax.spines.values():
            s.set_edgecolor(BLUE if big else "#666666")
            s.set_linewidth(2.6 if big else 1.1)
        ax.set_title(title, fontsize=13 if big else 11.5, fontweight="bold", pad=3)
        ax.set_xlabel(sub, fontsize=8.8, labelpad=4, color="#333333")
        pos[key] = rect
        lab, col = PROV[prov]
        fig.text(rect[0] + rect[2] - 0.006, rect[1] + rect[3] - 0.022, lab,
                 fontsize=7.6, ha="right", va="top", color="white", zorder=6,
                 bbox=dict(boxstyle="round,pad=0.30", fc=col, ec="none"))

    ov = fig.add_axes((0, 0, 1, 1)); ov.set_xlim(0, 1); ov.set_ylim(0, 1)
    ov.axis("off"); ov.patch.set_alpha(0); ov.set_zorder(0)

    for a, b, col, lw, lab in E:
        ra, rb = pos[a], pos[b]
        ca = (ra[0] + ra[2] / 2, ra[1] + ra[3] / 2)
        cb = (rb[0] + rb[2] / 2, rb[1] + rb[3] / 2)
        d = (cb[0] - ca[0], cb[1] - ca[1])
        p0 = _edge_point(ra, ca, d)
        p1 = _edge_point(rb, cb, (-d[0], -d[1]))
        ov.add_patch(FancyArrowPatch(p0, p1, arrowstyle="<|-|>", mutation_scale=15,
                                     lw=lw, color=col, shrinkA=0, shrinkB=0,
                                     zorder=1, alpha=0.95))
        lx, ly = p0[0] + (p1[0] - p0[0]) * 0.40, p0[1] + (p1[1] - p0[1]) * 0.40
        ov.text(lx, ly, lab, fontsize=10.5, color="#1a1a1a", ha="center", va="center",
                zorder=3, bbox=dict(boxstyle="round,pad=0.32", fc="white",
                                    ec=col, lw=1.3, alpha=0.97))

    rg = pos["grain"]
    ov.annotate("", xy=(rg[0] - 0.006, rg[1] + rg[3] * 0.74),
                xytext=(rg[0] - 0.006, rg[1] + rg[3] * 0.26),
                arrowprops=dict(arrowstyle="<|-|>", color=BLUE, lw=2.2,
                                connectionstyle="arc3,rad=1.3"))
    ov.text(rg[0] - 0.082, rg[1] + rg[3] / 2, "Grain-grain\ncollision", fontsize=10.5,
            ha="center", va="center",
            bbox=dict(boxstyle="round,pad=0.32", fc="white", ec=BLUE, lw=1.3))

    ov.plot([0.030, 0.068], [0.083, 0.083], color=BLUE, lw=3.2)
    ov.text(0.076, 0.083, "Two-way coupling  —  reaction returns to the body",
            fontsize=10.5, va="center")
    ov.plot([0.030, 0.068], [0.050, 0.050], color=GREY, lw=3.2)
    ov.text(0.076, 0.050, "Support only (ipc_only)  —  no reaction returned",
            fontsize=10.5, va="center")
    # 출처 배지 범례
    for i, k in enumerate(("xml", "stl", "py", "prim")):
        lab, col = PROV[k]
        ov.text(0.652 + i * 0.082, 0.074, lab, fontsize=8.2, ha="center",
                va="center", color="white",
                bbox=dict(boxstyle="round,pad=0.30", fc=col, ec="none"))
    ov.text(0.652, 0.042,
            "asset provenance  —  where each shape came from",
            fontsize=8.4, ha="left", va="center", color="#444444")

    ov.text(0.5, 0.012,
            "Every pair above is resolved by the same IPC coupler within a single "
            "world.advance(); only rigid-rigid pairs are disabled.",
            fontsize=10, ha="center", color="#444444")

    out = os.path.join(_HERE, "entity_graph.png")
    fig.savefig(out, facecolor="white", dpi=110)
    plt.close(fig)
    print("[saved]", out)


def _edge_point(rect, c, d):
    hw, hh = rect[2] / 2, rect[3] / 2
    if d[0] == 0 and d[1] == 0:
        return c
    s = min(hw / abs(d[0]) if d[0] else np.inf, hh / abs(d[1]) if d[1] else np.inf)
    return c[0] + d[0] * s, c[1] + d[1] * s


if __name__ == "__main__":
    main()

"""probe_mjcf_trees.py — MJCF body trees, one square PNG per mechanism.

2026-09-14. 사용자가 직접 만든 MJCF 5종(고정장치 / 회수장치2 / 공압(흡착)그리퍼 /
Crusher / RG-2)을 **같은 정사각형 크기**로 각각 한 장씩 낸다.

그림 규칙은 하나다:
    파랑 실선 화살표 = 자유도가 있는 조인트 (hinge=H / slide=S), 옆에 조인트 이름
    옅은 회색 선     = joint 없는 부착 = 용접 (Fusion 이 안 접어준 것)
    빨강 파선        = equality (weld 폐루프 / connect / joint mimic)

노드의 `g=N` 은 그 바디가 든 geom(=부품 메시) 수다. **바디 수만 세면 기구 규모를
오해한다** — Crusher 는 바디 8개지만 부품은 53개이고 그중 27개가 worldbody 직속
(=프레임에 용접)이라 바디로 안 잡힌다. 그래서 제목에 둘 다 적는다.

정사각형에 맞추는 법: 잎 노드 세로 간격(`ystep`)을 트리 가로폭 / 잎 수 로 잡아
트리가 저절로 정사각형에 가깝게 퍼지게 하고, 남는 쪽을 여백으로 채운다. 글자
크기는 데이터 범위에 반비례시켜 어느 기구든 같은 물리 크기로 읽히게 한다.

RG-2 는 `m0609_rg2_v2.xml` 안에 팔과 같이 들어 있으므로 `gripper_bracket`
서브트리만 떼어 그린다(팔 6축은 두산 제공분이라 제외).

Usage:  python probe_mjcf_trees.py
"""
import os
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

matplotlib.rcParams["font.family"] = "Malgun Gothic"     # 조인트 이름에 한글이 있다
matplotlib.rcParams["axes.unicode_minus"] = False

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE),
                                "Crusher_M0609_RG2_Tablet_Samplebag"))
import full_workflow as F                                     # noqa: E402
import paths                                                  # noqa: E402

BLUE, GREY, RED = "#0072B2", "#b9c0c7", "#d62728"
W = 1.20            # node box width  (data units)
H = 0.34            # node box height (data units)
DX = 1.62           # column pitch — W 보다 넉넉해야 세로 배선이 상자를 안 지난다


def _short(s, n=24):
    s = s or "?"
    return s if len(s) <= n else s[:n - 1] + "…"


def _find_body(wb, name):
    for b in wb.iter("body"):
        if b.get("name") == name:
            return b
    return None


def _layout(roots, ystep):
    nodes, edges, cnt = [], [], [0]

    def rec(body, depth, parent):
        kids = body.findall("body")
        if not kids:
            y = cnt[0] * ystep; cnt[0] += 1
        else:
            y = sum(rec(k, depth + 1, body) for k in kids) / len(kids)
        nodes.append(dict(name=body.get("name"), x=depth * DX, y=y,
                          g=len(body.findall("geom")),
                          joints=body.findall("joint")))
        edges.append((parent, body))
        return y

    for b in roots:
        rec(b, 0, None)
    return nodes, edges


def _n_leaves(roots):
    n = [0]

    def rec(b):
        k = b.findall("body")
        if not k:
            n[0] += 1
        for c in k:
            rec(c)
    for b in roots:
        rec(b)
    return max(n[0], 1)


def render(title, path, out, subtree=None, note="", counts_from_root=True):
    root = ET.parse(path).getroot()
    wb = root.find("worldbody")
    if subtree:
        sb = _find_body(wb, subtree)
        if sb is None:
            raise SystemExit("subtree %r not found in %s" % (subtree, path))
        roots, scope = [sb], sb
    else:
        roots, scope = wb.findall("body"), wb

    # 잎 간격을 가로폭에 맞춰 잡으면 트리가 저절로 정사각형에 가깝게 퍼진다.
    depth = 0

    def d(b, k=0):
        nonlocal depth
        depth = max(depth, k)
        for c in b.findall("body"):
            d(c, k + 1)
    for b in roots:
        d(b)
    xr = depth * DX + W + 1.5
    ystep = min(max(xr / _n_leaves(roots), 0.52), 1.05)
    nodes, edges = _layout(roots, ystep)
    by = {n["name"]: n for n in nodes}

    xs = [-1.20, max(n["x"] for n in nodes) + W + 0.25]
    ys = [-0.75, max(n["y"] for n in nodes) + 0.75]
    span = max(xs[1] - xs[0], ys[1] - ys[0])
    cx, cy = sum(xs) / 2, sum(ys) / 2

    fig = plt.figure(figsize=(9, 9))
    ax = fig.add_axes((0.02, 0.02, 0.96, 0.88))
    ax.set_xlim(cx - span / 2, cx + span / 2)
    ax.set_ylim(cy + span / 2, cy - span / 2)      # flipped
    ax.set_aspect("equal")
    ax.axis("off")
    k = min(9.0 / span, 1.05)                       # 글자 크기 스케일(상한)

    nb = len(list(scope.iter("body")))
    nj = len(list(scope.iter("joint")))
    ng = len(list(scope.iter("geom")))
    nw = len(wb.findall("geom")) if not subtree else 0
    head = "%s\n%d bodies · %d joints · %d parts" % (title, nb, nj, ng)
    if not subtree:
        head += " (%d welded to world)" % nw
    fig.suptitle(head + (("\n" + note) if note else ""),
                 fontsize=15, fontweight="bold", y=0.975, linespacing=1.6)

    # root marker
    ax.add_patch(FancyBboxPatch((-1.12, -H / 2), 0.78, H, boxstyle="round,pad=0.02",
                                fc="#eceff3", ec="#555555", lw=1.2, zorder=3))
    ax.text(-0.73, 0, "world" if not subtree else "link_6", fontsize=9 * k,
            ha="center", va="center", fontweight="bold", zorder=4)

    for parent, body in edges:
        n = by[body.get("name")]
        px, py = (-0.34, 0.0) if parent is None else (by[parent.get("name")]["x"] + W,
                                                      by[parent.get("name")]["y"])
        js = n["joints"]
        mov = bool(js)
        col = BLUE if mov else GREY
        lw = (1.8 if mov else 1.1) * min(k, 1.2)
        # 직교 3구간 배선: 부모 오른쪽 -> 열 사이 빈 칸에서 수직 -> 자식 왼쪽.
        # FancyArrowPatch 의 angle 연결선은 열을 가로질러 상자를 통과한다.
        mx = px + (n["x"] - px) * 0.38
        ax.plot([px, mx, mx, n["x"] - 0.05], [py, py, n["y"], n["y"]],
                color=col, lw=lw, solid_capstyle="round", zorder=1)
        ax.add_patch(FancyArrowPatch((n["x"] - 0.05, n["y"]), (n["x"], n["y"]),
                                     arrowstyle="-|>" if mov else "-",
                                     mutation_scale=9 * min(k, 1.2), lw=lw,
                                     color=col, shrinkA=0, shrinkB=0, zorder=1))
        if mov:
            j = js[0]
            tag = ("S " if j.get("type") == "slide" else "H ") + _short(j.get("name"), 22)
            ax.text(n["x"] + W / 2, n["y"] - H / 2 - 0.05, tag, fontsize=7.4 * k,
                    color=BLUE, ha="center", va="bottom", zorder=4)

    for n in nodes:
        mov = bool(n["joints"])
        ax.add_patch(FancyBboxPatch((n["x"], n["y"] - H / 2), W, H,
                                    boxstyle="round,pad=0.02",
                                    fc="#ffffff" if mov else "#f5f6f7",
                                    ec=BLUE if mov else GREY,
                                    lw=(1.6 if mov else 1.0) * min(k, 1.4), zorder=3))
        ax.text(n["x"] + W / 2, n["y"], "%s  g=%d" % (_short(n["name"], 20), n["g"]),
                fontsize=7.6 * k, ha="center", va="center", zorder=4,
                color="#111111" if mov else "#666666")

    eq = root.find("equality")
    if eq is not None:
        for e in eq:
            if e.tag in ("weld", "connect"):
                a, b = by.get(e.get("body1")), by.get(e.get("body2"))
                if not (a and b):
                    continue
                ax.add_patch(FancyArrowPatch((a["x"] + W / 2, a["y"]),
                                             (b["x"] + W / 2, b["y"]),
                                             arrowstyle="-", lw=1.8 * min(k, 1.4),
                                             color=RED, linestyle=(0, (4, 3)),
                                             connectionstyle="arc3,rad=-0.35", zorder=2))
                ax.text((a["x"] + b["x"]) / 2 + W / 2,
                        (a["y"] + b["y"]) / 2 - 0.45, "%s  %s" % (e.tag, _short(e.get("name"), 22)),
                        fontsize=7.4 * k, color=RED, ha="center", zorder=4)
        mim = [e for e in eq if e.tag == "joint" and e.get("joint2")]
        if mim and (not subtree or any(by.get(m.get("joint1")) is None for m in mim)):
            # y 축이 뒤집혀 있다 — 화면 아래쪽은 cy + span/2 다. 위에 두면 제목을 덮는다.
            ax.text(cx + span / 2 - 0.15, cy + span / 2 - 0.80,
                    "equality joint (mimic) x %d  ->  %s" % (len(mim), mim[0].get("joint2")),
                    fontsize=8.6 * k, color=RED, ha="right", va="bottom")

    ax.text(cx - span / 2 + 0.15, cy + span / 2 - 0.15,
            "blue = movable joint    grey = welded attachment    "
            "red dashed = equality\ng=N : geom (part meshes) on that body",
            fontsize=8.2 * k, color="#555555", ha="left", va="bottom", linespacing=1.5)

    fig.savefig(out, facecolor="white", dpi=150)
    plt.close(fig)
    print("[saved] %-28s  %2d bodies / %2d joints / %3d parts" % (os.path.basename(out), nb, nj, ng))


def main():
    suction = paths.ascii_safe_mjcf(
        os.path.join(paths.ROBOTS_DIR, "석션V1_description", "석션V1.xml"))
    out = os.path.join(_HERE, "mjcf_tree_%s.png")
    render("Fixture (고정장치)", F.FIXTURE_MJCF, out % "fixture",
           note="single hinge — the servo shaft carries 66 of the 90 parts")
    render("Recovery unit 2 (회수장치2)", F.RECOVERY2_MJCF, out % "recovery",
           note="33 bodies but only 3 joints — 30 are welded decoration")
    render("Suction gripper V1 (공압그리퍼)", suction, out % "suction",
           note="stage + two cup jaws; the right jaw is a mimic of the left")
    render("Crusher", F.CRUSHER_SRC_XML, out % "crusher",
           note="crank-slider closed loop is held by a weld\n"
                "(source also has 'equality joint lock_crank' — removed at load)")
    render("RG-2 gripper", F.ROBOT_MJCF, out % "rg2", subtree="gripper_bracket",
           note="subtree of m0609_rg2_v2.xml below link_6\n"
                "1 real DOF (gripper_joint); the other 5 joints are mimics")


if __name__ == "__main__":
    main()

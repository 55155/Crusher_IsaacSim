"""probe_waypoints.py — full_workflow 가 찍는 **웨이포인트 좌표를 그대로 출력**한다
(2026-09-14, 사용자 요청 "웨이포인트 좌표를 실제로 찍어서 보여줘").

씬을 안 띄운다. `full_workflow` 를 import 만 하면 되는데, 이 파일의 모듈 레벨은
genesis 를 안 끌어오고(지연 임포트) `slot_geometry()` 도 "STL + CRUSHER_POS/EULER
만으로 정해지므로 씬 없이 계산된다"고 스스로 밝혀 둔 함수라서다. 따라서 여기서
찍는 값은 재구현이 아니라 **런이 쓰는 바로 그 상수·그 함수**의 출력이다.

손값 자세(Q_GRASP/Q_LIFT)의 TCP 는 **MuJoCo FK** 로 푼다 — 같은 MJCF 를 읽으므로
Genesis 를 띄우는 것과 같은 기구학이다. 검산: Q_GRASP FK 가 코드 상수
`FINGER_MID_BASE` 와 0.004mm 안에서 일치하고, Q_LIFT FK z 가 런 로그의
`[phase] lift @done finger_z` 와 0.07mm 안에서 일치한다.

한계(정직하게):
  - 카테시안 웨이포인트(insert/extract/rcdown, 경유점)는 전량 재현된다.
  - above/insert 의 **관절각**은 IK 가 필요하다 — 여기선 안 푼다(Genesis 필요).
    실제 런 로그의 `[ik] ... arm_q=` / `[phase] ... finger_z=` 로 대조만 한다.
  - toRC 120점은 RRTConnect 가 만드는 **관절공간** 경로라 결정적이지 않다.
    좌표로 찍을 수 있는 건 폴백 경유점 3점뿐이다.

사용법:
    python probe_waypoints.py [런로그.log]
"""
import os
import re
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE),
                                "Crusher_M0609_RG2_Tablet_Samplebag"))
import full_workflow as F                                    # noqa: E402

MM = 1e3


def _row(name, p, extra=""):
    print(f"  {name:<10s} ({p[0]*MM:+9.2f}, {p[1]*MM:+9.2f}, {p[2]*MM:+9.2f})  {extra}")


def _fk_tcp():
    """Q_GRASP/Q_LIFT 의 TCP world 좌표 — 같은 MJCF 를 MuJoCo 로 FK 한다.

    Genesis 는 로봇을 ROBOT_OFFSET 에 스폰하므로 base 프레임 결과에 그걸 더한다.
    핑거 6 DOF 는 full_workflow 와 같이 FING_CLOSE 로 전부 같은 값을 준다.
    """
    try:
        import mujoco
    except ImportError:
        return None
    m = mujoco.MjModel.from_xml_path(F.ROBOT_MJCF)
    d = mujoco.MjData(m)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, F.FINGER_LINKS[0])
    out = {}
    for nm, q in (("grasp", F.Q_GRASP), ("lift", F.Q_LIFT)):
        d.qpos[:6], d.qpos[6:12] = q, F.FING_CLOSE
        mujoco.mj_forward(m, d)
        out[nm] = (d.xpos[bid] + d.xmat[bid].reshape(3, 3) @ F.FINGER_TCP_LOCAL
                   + F.ROBOT_OFFSET)
    return out


def _box(ax, lo, hi, **kw):
    """AABB 를 와이어프레임으로 — 3D/2D 축 양쪽에서 쓴다."""
    lo, hi = np.asarray(lo) * MM, np.asarray(hi) * MM
    c = np.array([[lo[0], lo[1], lo[2]], [hi[0], lo[1], lo[2]], [hi[0], hi[1], lo[2]],
                  [lo[0], hi[1], lo[2]], [lo[0], lo[1], hi[2]], [hi[0], lo[1], hi[2]],
                  [hi[0], hi[1], hi[2]], [lo[0], hi[1], hi[2]]])
    ed = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
          (0, 4), (1, 5), (2, 6), (3, 7)]
    for a, b in ed:
        if hasattr(ax, "plot3D"):
            ax.plot(*zip(c[a], c[b]), **kw)
            kw.pop("label", None)
        else:
            i, j = ax._proj                      # 2D 뷰가 쓰는 축 쌍
            ax.plot([c[a][i], c[b][i]], [c[a][j], c[b][j]], **kw)
            kw.pop("label", None)


def _plot3d(sg, target_xy, above_z, insert_z, fk, rc_xy, rc_above_z, rc_finger_z,
            p_mid, rc_x, rc_y, rc_jaw_top_z):
    """웨이포인트를 3D + 상면/정면으로 그린다.

    **카테시안 사다리(실선)와 관절공간 구간(점선)을 반드시 구분한다.** 후자는
    두 끝점만 아는 것이고 그 사이 TCP 가 실제로 그리는 곡선은 여기서 계산하지
    않았다 — 직선으로 이어 놓고 경로라고 하면 거짓말이 된다.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    matplotlib.rcParams["font.family"] = "Malgun Gothic"
    matplotlib.rcParams["axes.unicode_minus"] = False

    hang = F.BAG_HANG_BELOW_FINGER
    # (순번, 이름, 좌표, 폴백전용인가) — 순번은 실행 순서다.
    P = [
        ("1", "grasp", np.r_[fk["grasp"]] if fk else np.r_[F.FINGER_MID], False),
        ("2", "lift", np.r_[fk["lift"]] if fk else np.r_[F.FINGER_MID + [0, 0, .126]], False),
        ("3", "above", np.r_[target_xy, above_z], False),
        ("4", "insert", np.r_[target_xy, insert_z], False),
        ("6", "경유", np.asarray(p_mid), True),
        ("7", "rc진입", np.r_[rc_xy, rc_above_z], False),
        ("8", "rcdown", np.r_[rc_xy, rc_finger_z], False),
    ]
    ins = np.c_[np.tile(target_xy, (41, 1)), np.linspace(above_z, insert_z, 41)]
    dwn = np.c_[np.tile(rc_xy, (F.RECOVER_DOWN_WAYS, 1)),
                np.linspace(rc_above_z, rc_finger_z, F.RECOVER_DOWN_WAYS)]
    _p = dict((n, v) for _, n, v, _ in P)

    # (구간, 점열, 카테시안인가, 색, 라벨)
    # insert 와 extract 는 **같은 41점 사다리**를 내려갔다 올라오는 것이라 겹친다 —
    # 두 번 그리면 하나가 다른 하나를 덮을 뿐이므로 한 번만 그리고 라벨에 적는다.
    segs = [
        ("lift", np.array([_p["grasp"], _p["lift"]]), False, "#999999", None),
        ("above", np.array([_p["lift"], _p["above"]]), False, "#999999", None),
        ("insert", ins, True, "#0072B2", "insert(4) / extract(6) — 같은 41점 사다리"),
        ("toRC", np.array([_p["above"], _p["경유"], _p["rc진입"]]), False, "#999999",
         "관절공간 — 끝점만 안다(실제 TCP 경로 아님)"),
        ("rcdown", dwn, True, "#D55E00", f"rcdown {F.RECOVER_DOWN_WAYS}점 (카테시안)"),
        ("home", np.array([_p["rcdown"], _p["grasp"]]), False, "#999999", None),
    ]
    wb = F.crusher_mesh_world_aabb(F.WALL_BACK_MESH)
    wl = F.crusher_mesh_world_aabb(F.WALL_LEFT_MESH, F.LEFTWALL_BODY_POS,
                                   F.LEFTWALL_GEOM_POS)

    def draw(ax, is3d):
        for nm, pts, cart, col, lb in segs:
            p = np.asarray(pts) * MM
            a = (p[:, 0], p[:, 1], p[:, 2]) if is3d else (p[:, ax._proj[0]],
                                                          p[:, ax._proj[1]])
            ax.plot(*a, color=col, lw=1.8 if cart else 1.0,
                    ls="-" if cart else ":", label=lb, zorder=4)
            if cart:
                ax.scatter(*a, color=col, s=9, zorder=5)
        # 봉투 하단 — 맞춰야 하는 건 TCP 가 아니라 이쪽이다. insert 끝에서 이 선이
        # wall_center_z 에 정확히 닿는 게 목표 정의 그 자체다(insert_z 역산).
        _lb = [f"봉투 하단 (TCP-{hang*MM:.0f}mm) — 실제 맞춤 대상"]
        for nm, pts, cart, col, _ in segs:
            if not cart:
                continue
            p = np.asarray(pts) * MM
            p = np.c_[p[:, 0], p[:, 1], p[:, 2] - hang * MM]
            a = (p[:, 0], p[:, 1], p[:, 2]) if is3d else (p[:, ax._proj[0]],
                                                          p[:, ax._proj[1]])
            ax.plot(*a, color="#E69F00", lw=1.5, ls="--", zorder=3,
                    label=(_lb.pop() if (_lb and is3d) else None))
        _box(ax, *wb, color="#444444", lw=0.8, alpha=0.8,
             label="Wall3 / Left_Wall (슬롯)" if is3d else None)
        _box(ax, *wl, color="#444444", lw=0.8, alpha=0.8)
        jaw = np.array([[rc_x, rc_y, F.RECOVERY2_POS[2] + 0.048],
                        [rc_x, rc_y, rc_jaw_top_z]]) * MM
        a = ((jaw[:, 0], jaw[:, 1], jaw[:, 2]) if is3d
             else (jaw[:, ax._proj[0]], jaw[:, ax._proj[1]]))
        ax.plot(*a, color="#CC79A7", lw=4, alpha=0.7, solid_capstyle="butt",
                label="회수장치 턱 (로컬 z 48~148mm)" if is3d else None, zorder=2)
        base = np.array([F.ROBOT_OFFSET[0], F.ROBOT_OFFSET[1], 0.0]) * MM
        a = (base[0], base[1], base[2]) if is3d else (base[ax._proj[0]], base[ax._proj[1]])
        ax.scatter(*a, marker="s", s=55, color="#009E73", zorder=6,
                   label="로봇 베이스" if is3d else None)
        # 라벨이 겹치면 못 읽는다 — 이름마다 방향을 달리 준다(2D 뷰 기준 오프셋).
        off = {"grasp": (10, -16), "lift": (10, 8), "above": (10, 10),
               "insert": (10, -14), "경유": (8, 10), "rc진입": (-52, 12),
               "rcdown": (-52, -16)}
        for k, nm, p, fb in P:
            q = np.asarray(p) * MM
            a = (q[0], q[1], q[2]) if is3d else (q[ax._proj[0]], q[ax._proj[1]])
            ax.scatter(*a, marker="X" if fb else "o", s=40,
                       facecolor="#bbbbbb" if fb else "white", edgecolor="black",
                       linewidths=1.0, zorder=7)
            tx = f"{k} {nm}" + ("(폴백)" if fb else "")
            if is3d:
                ax.text(*a, "  " + tx, fontsize=8, zorder=8)
            else:
                ax.annotate(tx, a, textcoords="offset points", xytext=off.get(nm, (8, 8)),
                            fontsize=8, zorder=8)

    fig = plt.figure(figsize=(19, 10.2))
    gs_ = fig.add_gridspec(2, 3, width_ratios=[1.5, 1, 1], wspace=0.26, hspace=0.28)
    fig.suptitle("full_workflow 팔 웨이포인트 — TCP(f1-f2 중앙) world 좌표 [mm]   "
                 "실선 = 카테시안 사다리 · 점선 = 관절공간(끝점만) · "
                 f"주황 파선 = 봉투 하단(TCP-{hang*MM:.0f}mm)", fontsize=12)
    ax = fig.add_subplot(gs_[:, 0], projection="3d")
    draw(ax, True)
    allp = np.concatenate([np.asarray(p) for _, p, _, _, _ in segs]) * MM
    lo, hi = allp.min(axis=0) - 40, allp.max(axis=0) + 40
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(min(lo[2], 0), hi[2])
    ax.set_box_aspect((hi - lo)[0:3] / (hi - lo).max())
    ax.view_init(elev=22, azim=-58)
    ax.set_xlabel("x [mm]"); ax.set_ylabel("y [mm]"); ax.set_zlabel("z [mm]")
    ax.legend(fontsize=8, loc="upper left")
    ax.set_title("등각", fontsize=10)

    wcz = sg["wall_center_z"] * MM
    tx, ty = target_xy * MM
    views = (
        ((0, 1), (0, 1), "상면도 (X-Y)", "x [mm]", "y [mm]", None),
        ((1, 1), (0, 2), "정면도 (X-Z)", "x [mm]", "z [mm]", None),
        ((0, 2), (0, 2), "슬롯 확대 — insert 사다리 하단", "x [mm]", "z [mm]",
         (tx - 55, tx + 25, wcz - 25, insert_z * MM + 45)),
        ((1, 2), (0, 2), f"회수장치 확대 — rcdown {F.RECOVER_DOWN_WAYS}점", "x [mm]",
         "z [mm]", (rc_xy[0] * MM - 50, rc_xy[0] * MM + 30,
                    rc_finger_z * MM - 105, rc_above_z * MM + 25)),
    )
    for cell, proj, tt, xl, yl, lim in views:
        a = fig.add_subplot(gs_[cell])
        a._proj = proj
        draw(a, False)
        a.set_aspect("equal"); a.grid(alpha=0.3)
        a.set_title(tt, fontsize=10); a.set_xlabel(xl); a.set_ylabel(yl)
        if lim:
            a.set_xlim(lim[0], lim[1]); a.set_ylim(lim[2], lim[3])
        # wall_center_z 는 슬롯 얘기다 — 축 범위 밖(회수장치 확대)에 그리면
        # 선은 안 보이는데 라벨만 패널 밖에 떠서 오해를 부른다.
        if proj[1] == 2 and a.get_ylim()[0] <= wcz <= a.get_ylim()[1]:
            a.axhline(wcz, color="#d62728", ls=":", lw=1.0)
            a.text(a.get_xlim()[0], wcz, f" wall_center_z {wcz:.0f}", fontsize=7,
                   color="#d62728", va="bottom")
        if lim and cell == (0, 2):
            # 목표 정의 그 자체 — insert 끝에서 봉투 하단이 이 선에 닿는다.
            a.annotate(f"봉투 하단이 여기 닿는다\n(insert_z {insert_z*MM:.0f} - "
                       f"{hang*MM:.0f} = {wcz:.0f})", (tx, wcz), fontsize=7.5,
                       color="#E69F00", textcoords="offset points", xytext=(-120, 22),
                       arrowprops=dict(arrowstyle="->", color="#E69F00", lw=1.0))
        if lim and cell == (1, 2):
            a.annotate(f"봉투 하단 = F_Top 상면 -{F.RC_BITE_DEPTH*MM:.0f}mm",
                       (rc_xy[0] * MM, (rc_finger_z - hang) * MM), fontsize=7.5,
                       color="#E69F00", textcoords="offset points", xytext=(-130, -24),
                       arrowprops=dict(arrowstyle="->", color="#E69F00", lw=1.0))
    out = os.path.join(_HERE, "waypoints_3d.png")
    fig.savefig(out, facecolor="white", dpi=115, bbox_inches="tight")
    plt.close(fig)
    print(f"\n[saved] {out}")


def main():
    sg = F.slot_geometry()
    gap_cx, gap_cy = sg["gap_cx"], sg["gap_cy"]
    wall_top_z, wall_center_z = sg["wall_top_z"], sg["wall_center_z"]

    # ── main() Phase 7 과 **같은 식**으로 목표를 역산한다 ────────────────
    above_z = wall_top_z + 0.20
    insert_z = wall_center_z + F.BAG_HANG_BELOW_FINGER
    target_xy = np.array([gap_cx - F.BAG_DX_FROM_FINGER,
                          gap_cy - F.BAG_DY_FROM_FINGER + F.Y_OFFSET])

    print("=" * 78)
    print("슬롯 기하 — 두 벽 STL 의 world AABB 에서 나온다 (사람이 안 찍는다)")
    print("=" * 78)
    print(f"  gap 중심      ({gap_cx*MM:+.2f}, {gap_cy*MM:+.2f}) mm   "
          f"gap 폭 {sg['gap_width']*MM:.1f}mm  (간격축 = world "
          f"{'XY'[sg['gap_ax']]})")
    print(f"  wall_top_z    {wall_top_z*MM:+.2f} mm     "
          f"wall_center_z {wall_center_z*MM:+.2f} mm")
    print(f"  봉투중심 보정  dx={F.BAG_DX_FROM_FINGER*MM:+.1f}mm  "
          f"dy={F.BAG_DY_FROM_FINGER*MM:+.1f}mm  (파지 GRIP_OFFSET_MM="
          f"{F.GRIP_OFFSET_MM:+.0f}) -> TCP 타깃이 그만큼 밀린다")

    print()
    print("=" * 78)
    print("팔 웨이포인트 — TCP(f1-f2 중앙) world 좌표 [mm]")
    print("=" * 78)
    print("  이름           x          y          z        비고")
    fk = _fk_tcp()
    if fk is None:
        _row("grasp", F.FINGER_MID, "Q_GRASP (mujoco 없어 상수 FINGER_MID 사용)")
        print(f"  {'lift':<10s} (        ?,         ?,         ?)  Q_LIFT — mujoco 필요")
    else:
        _row("grasp", fk["grasp"], f"Q_GRASP FK (상수 FINGER_MID 와 "
             f"{np.abs(fk['grasp']-F.FINGER_MID).max()*MM:.3f}mm 일치), 스텝 "
             f"{F.N_DROP}+{F.N_SETTLE}+{F.N_CLOSE}+{F.N_GRASP}")
        _row("lift", fk["lift"], f"Q_LIFT FK — grasp 에서 +"
             f"{(fk['lift'][2]-fk['grasp'][2])*MM:.1f}mm 수직상승, 스텝 "
             f"{F.N_LIFT}+{F.N_HOLD}")
    _row("above", np.r_[target_xy, above_z],
         f"wall_top_z+200mm, 관절보간 2점, 스텝 {F.N_ABOVE}(+{F.N_ABOVE_SETTLE} 정지)")
    _row("insert", np.r_[target_xy, insert_z],
         f"wall_center_z+{F.BAG_HANG_BELOW_FINGER*MM:.0f}mm, **41점**, 스텝 {F.N_INSERT}")

    # ── 회수장치 쪽 목표 (main() Phase 12 와 같은 식) ────────────────────
    rc_x = F.RECOVERY2_POS[0] + F.RC_JAW_X
    rc_y = F.RECOVERY2_POS[1] + F.RC_LINK_MID_Y
    rc_seal_z = F.RECOVERY2_POS[2] + F.RC_PLATE_TOP_Z - F.RC_BITE_DEPTH
    rc_finger_z = rc_seal_z + F.BAG_HANG_BELOW_FINGER
    rc_jaw_top_z = F.RECOVERY2_POS[2] + 0.148
    rc_above_z = max(rc_finger_z + F.RECOVER_APPROACH_H,
                     rc_jaw_top_z + F.BAG_HANG_BELOW_FINGER + F.RECOVER_CLEAR_Z)
    # 봉투중심 보정 — 손목을 RC_WRIST_DEG 돌린 방향으로 회전시켜 뺀다.
    _rad = np.radians(F.RC_WRIST_DEG)
    rc_tcp_x = rc_x - np.cos(_rad) * F.BAG_DX_FROM_FINGER
    rc_tcp_y = rc_y - np.sin(_rad) * F.BAG_DX_FROM_FINGER

    _row("extract", np.r_[target_xy, above_z],
         f"insert 사다리 역주행 41점, 스텝 {F.N_EXTRACT}")
    p_mid = np.array([(target_xy[0] + rc_tcp_x) / 2.0,
                      (target_xy[1] + rc_tcp_y) / 2.0,
                      max(above_z, rc_above_z) + F.RECOVER_CLEAR_H])
    _row("toRC경유", p_mid, f"**폴백 전용** — 기본은 RRTConnect {F.RC_PLAN_WAYS}점"
                            f"(관절공간), 스텝 {F.N_TO_RC}")
    _row("rc진입", np.array([rc_tcp_x, rc_tcp_y, rc_above_z]),
         f"턱 상단 {rc_jaw_top_z*MM:.1f}mm 에서 역산")
    _row("rcdown", np.array([rc_tcp_x, rc_tcp_y, rc_finger_z]),
         f"봉투하단이 F_Top-{F.RC_BITE_DEPTH*MM:.0f}mm, "
         f"**{F.RECOVER_DOWN_WAYS}점**, 스텝 {F.N_RC_DOWN}")
    print(f"  {'home':<10s} 관절공간 Q_GRASP 복귀 — 스텝 {F.N_HOME}")
    print(f"  (물림 중심은 ({rc_x*MM:+.2f}, {rc_y*MM:+.2f})mm, TCP 는 봉투중심 보정으로 "
          f"{np.hypot(rc_tcp_x-rc_x, rc_tcp_y-rc_y)*MM:.1f}mm 밀려 있다)")

    # ── 사다리 전량 ──────────────────────────────────────────────────────
    print()
    print("=" * 78)
    print(f"insert 사다리 — solve_descent_waypoints(z {above_z*MM:.1f} -> "
          f"{insert_z*MM:.1f}mm, n=41). xy 고정, z 만 균등")
    print("=" * 78)
    zs = np.linspace(above_z, insert_z, 41)
    for i in range(0, 41, 4):
        line = "  ".join(f"#{j:02d} z={zs[j]*MM:7.2f}" for j in range(i, min(i + 4, 41)))
        print("  " + line)
    print(f"  간격 {abs(zs[1]-zs[0])*MM:.2f}mm — 41점이라 구간 내 카테시안 부풂이 "
          f"0.01mm 수준으로 죽는다(2점 보간이면 중간에서 dy=+9.67mm)")

    print()
    print("=" * 78)
    print(f"rcdown 사다리 — n={F.RECOVER_DOWN_WAYS}, z {rc_above_z*MM:.1f} -> "
          f"{rc_finger_z*MM:.1f}mm  (xy 고정 {rc_tcp_x*MM:+.2f}, {rc_tcp_y*MM:+.2f})")
    print("=" * 78)
    for j, z in enumerate(np.linspace(rc_above_z, rc_finger_z, F.RECOVER_DOWN_WAYS)):
        print(f"  #{j}  ({rc_tcp_x*MM:+8.2f}, {rc_tcp_y*MM:+8.2f}, {z*MM:+8.2f})  "
              f"봉투하단 {(z - F.BAG_HANG_BELOW_FINGER)*MM:+7.2f}")

    print()
    print("=" * 78)
    print("웨이포인트 -> 스텝: run_arm_path 가 u = ease((k+1)/N)*m 로 **사이를 다시** 보간")
    print("=" * 78)
    for nm, m, n in (("insert", 41, F.N_INSERT), ("extract", 41, F.N_EXTRACT),
                     ("toRC", F.RC_PLAN_WAYS, F.N_TO_RC),
                     ("rcdown", F.RECOVER_DOWN_WAYS, F.N_RC_DOWN)):
        print(f"  {nm:<8s} 웨이포인트 {m:3d} / 스텝 {n:4d} = 웨이포인트당 "
              f"{n/(m-1):6.1f}스텝  {'OK' if m < n else '**보간 없음 — 봉투가 뜯긴다**'}")

    _plot3d(sg, target_xy, above_z, insert_z, fk,
            np.array([rc_tcp_x, rc_tcp_y]), rc_above_z, rc_finger_z, p_mid,
            rc_x, rc_y, rc_jaw_top_z)

    # ── 실제 런 로그와 대조 ──────────────────────────────────────────────
    log = sys.argv[1] if len(sys.argv) > 1 else None
    if log and os.path.exists(log):
        print()
        print("=" * 78)
        print(f"실제 런 대조 — {os.path.basename(log)}")
        print("=" * 78)
        txt = open(log, encoding="utf-8", errors="replace").read()
        cmd = {"above": above_z, "insert": insert_z, "extract": above_z,
               "toRC": rc_above_z, "rcdown": rc_finger_z}
        # 회수장치 z 는 RC_BITE_DEPTH 기본값이 바뀌면 같이 움직인다(10 -> 5mm).
        # 그 차이를 추종오차로 둔갑시키지 않도록 **로그가 찍은 목표**를 우선한다.
        _m = re.search(r"\[recover\] 물림 중심.*?finger z=([\d.]+)", txt)
        if _m and abs(float(_m.group(1)) - rc_finger_z) > 1e-4:
            print(f"  [주의] 이 로그의 rcdown 목표는 {float(_m.group(1))*MM:.1f}mm "
                  f"— 지금 기본값 {rc_finger_z*MM:.1f}mm 과 다르다"
                  f"(RC_BITE_DEPTH 변경). 로그 값으로 대조한다.")
            cmd["rcdown"] = float(_m.group(1))
        _m = re.search(r"\[recover\] 턱 상단.*?진입 finger z=([\d.]+)", txt)
        if _m:
            cmd["toRC"] = float(_m.group(1))
        print(f"  {'phase':<10s} {'지령 z':>9s} {'실측 finger_z':>14s} {'오차':>9s}")
        for m in re.finditer(r"\[phase\] (\w+)\s+@done.*?finger_z=([\d.]+)", txt):
            nm, z = m.group(1), float(m.group(2))
            if nm in cmd:
                print(f"  {nm:<10s} {cmd[nm]*MM:9.2f} {z*MM:14.2f} "
                      f"{(z-cmd[nm])*MM:+9.2f} mm")
        for m in re.finditer(r"\[ik\] (\S+) target=\[([^\]]+)\]", txt):
            v = np.fromstring(m.group(2), sep=" ")
            print(f"  [로그] IK 타깃 {m.group(1):<12s} "
                  f"({v[0]*MM:+.2f}, {v[1]*MM:+.2f}, {v[2]*MM:+.2f}) mm")
    elif log:
        print(f"\n[경고] 로그를 못 찾았다: {log}")


if __name__ == "__main__":
    main()

"""
build_leftwall_split.py — Left_Wall 충돌 형상을 "본체 + 플랜지" 로 분리해 굽는다.

**왜 필요한가 (2026-08-26 실측)**

`full_workflow.py` 는 Crusher 를 `gs.morphs.MJCF(..., convexify=True)` 로 싣는다.
Genesis 는 MJCF 를 robot 으로 보고 `decompose_robot_error_threshold` 를 쓰는데
그 기본값이 **inf** 다(genesis/options/morphs.py). 임계값이 무한대면
`_postprocess_collision_geoms_impl` 의 `geoms_must_decompose` 가 영원히 False 라
**볼록분해가 한 번도 일어나지 않고 메시마다 hull 이 딱 하나** 생긴다.

L2_Left_Wall1_1 은 ㅁ 프레임이라 이게 치명적이다:

    실부피      38,123 mm^3
    convex hull 136,890 mm^3   -> hull 대비 실부피 27.8%, 허수 부피 98,767 mm^3

그 허수 부피가 바로 **좌우 세로부재 사이 55mm 개구부** 다. impact plate
(L9_PLATE_v3_1)가 지나가며 봉투를 분쇄하는 통로가 통째로 막힌다 —
`_add_leftwall_clamp_face()` 의 box 처방이 폐기된 이유(§18-2)와 같은 실패다.

옆에 있는 `L2_Left_Wall1_1_hull_000~010.stl` 은 예전에 CoACD 로 구워 둔 것인데
**XML 어디에서도 참조하지 않는다**. 즉 지금까지 런타임 형상은 저 hull 11개가
아니라 통짜 hull 하나였다.

**처방**

사용자가 Fusion 에서 벽을 두 부품으로 쪼개 다시 내보냈다
(`Crusher_IsaacSim_mjcf/meshes/{3_Left_Wall_1, 2_Left_Wall1_1}.stl`).
그 둘만 가져와 현재 모델 좌표계로 옮기고, 본체는 CoACD 로 더 쪼갠다.
융합(fusion) 분기는 `must_decompose` 가 False 라 통째로 스킵되므로
**geom 을 나눠 두면 나뉜 채로 유지된다** — 이 이식이 성립하는 근거다.

플랜지는 verts 8개짜리 완전한 직육면체(18x5x65mm)라 hull == 실부피 100%,
볼록화 오차가 원리적으로 0 이다. 그대로 쓴다.

**좌표 변환**

두 export 는 축 방향이 완전히 같고 원점만 다르다. Wall1/Wall2/Wall3/PLATE 의
치수가 소수점까지 일치하는 것으로 확인했다. 오프셋은 하드코딩하지 않고 아래에서
공통 부품으로 유도한 뒤 나머지 부품으로 교차검증한다(불일치하면 즉시 죽는다).

주의: 새 CAD 의 Left_Wall 은 그 변환을 적용해도 예전 위치보다 **x 로 0.50mm
더 물러나** 있다. 압착 간격이 플랜지 12.00 -> 12.50mm, 본체 17.00 -> 17.50mm 로
바뀐다. 의도한 CAD 수정이라 보고 그대로 반영한다 — full_workflow.py 의
LEFTWALL_GAP_* 상수가 이 값을 물고 있다.

실행:  python build_leftwall_split.py
"""
import os
import sys

import numpy as np
import trimesh
import coacd

HERE = os.path.dirname(os.path.abspath(__file__))
NEW_EXPORT = os.environ.get(
    "LEFTWALL_NEW_EXPORT",
    r"C:/Users/user/Downloads/Crusher_IsaacSim_mjcf/meshes",
)

# 신규 export 에서 가져올 두 조각. 이름은 Fusion 쪽 body 이름 그대로다.
SRC_BODY = "3_Left_Wall_1"      # ㅁ 프레임 본체 (164 verts, 오목)
SRC_FLANGE = "2_Left_Wall1_1"   # 5mm 앞선 상단 플랜지 (8 verts, 정확한 box)

OUT_BODY = "L2_Left_Wall_body.stl"              # 본체 원형 — 시각 전용 geom 이 쓴다
OUT_BODY_HULL = "L2_Left_Wall_body_hull_{:03d}.stl"   # 본체 충돌 — CoACD hull
OUT_FLANGE = "L2_Left_Wall_flange.stl"          # 플랜지 — 시각/충돌 겸용(정확한 box)

# 압착 간격을 재는 기준면: 고정 3벽의 마주보는 면(전부 같은 평면).
FIXED_WALL_REF = "L2_Wall3_1"

# 두 export 에 모두 있고 형상이 동일한 부품 — 좌표 오프셋 유도/검증용.
# (신규 이름, 현재 이름)
COMMON_PARTS = [
    ("2_Wall3_1", "L2_Wall3_1"),
    ("1_Wall1_1", "L1_Wall1_1"),
    ("1_Wall2_1", "L1_Wall2_1"),
    ("9_PLATE_v3_1", "L9_PLATE_v3_1"),
]

COACD_THRESHOLD = float(os.environ.get("LEFTWALL_COACD_THRESHOLD", "0.05"))


def load(path):
    m = trimesh.load(path, force="mesh")
    if isinstance(m, trimesh.Scene):
        m = trimesh.util.concatenate(m.dump())
    return m


def derive_offset():
    """신규 export root 좌표 -> 현재 모델 STL 좌표 평행이동을 공통 부품에서 유도한다.

    첫 부품으로 오프셋을 잡고 나머지로 검증한다. 축이 어긋났거나 다른 리비전이면
    여기서 불일치가 나므로, 조용히 틀어진 모델이 만들어지는 일은 없다.
    """
    offs = []
    for new_name, cur_name in COMMON_PARTS:
        p_new = os.path.join(NEW_EXPORT, new_name + ".stl")
        p_cur = os.path.join(HERE, cur_name + ".stl")
        if not (os.path.exists(p_new) and os.path.exists(p_cur)):
            print(f"  [skip] {new_name} / {cur_name} — 파일 없음")
            continue
        v_new, v_cur = load(p_new).vertices, load(p_cur).vertices
        s_new, s_cur = v_new.max(0) - v_new.min(0), v_cur.max(0) - v_cur.min(0)
        if not np.allclose(s_new, s_cur, atol=1e-3):
            raise SystemExit(
                f"[중단] {new_name} 와 {cur_name} 의 치수가 다르다 "
                f"{s_new} vs {s_cur} — 두 export 가 같은 리비전이 아니다."
            )
        off = v_cur.min(0) - v_new.min(0)
        print(f"  {cur_name:16s} 오프셋 {off[0]:+9.3f} {off[1]:+9.3f} {off[2]:+9.3f} mm")
        offs.append(off)

    if not offs:
        raise SystemExit("[중단] 공통 부품을 하나도 찾지 못했다.")
    offs = np.array(offs)
    spread = offs.max(0) - offs.min(0)
    if spread.max() > 1e-3:
        raise SystemExit(
            f"[중단] 공통 부품들이 일관된 평행이동을 주지 않는다 (편차 {spread} mm) — "
            "축 방향이 다르거나 리비전이 섞였다."
        )
    return offs.mean(0)


def main():
    print("=" * 74)
    print("[1/4] 좌표 오프셋 유도 (신규 export root -> 현재 모델 STL 프레임)")
    print("=" * 74)
    off = derive_offset()
    print(f"  => 채택 {off[0]:+.3f} {off[1]:+.3f} {off[2]:+.3f} mm\n")

    body = load(os.path.join(NEW_EXPORT, SRC_BODY + ".stl"))
    flange = load(os.path.join(NEW_EXPORT, SRC_FLANGE + ".stl"))
    body.apply_translation(off)
    flange.apply_translation(off)

    print("=" * 74)
    print("[2/4] 옮긴 결과 — 예전 통짜 메시와 대조")
    print("=" * 74)
    old = load(os.path.join(HERE, "L2_Left_Wall1_1.stl"))
    for tag, m in (("본체", body), ("플랜지", flange), ("(예전) 통짜", old)):
        lo, hi = m.vertices.min(0), m.vertices.max(0)
        print(f"  {tag:10s} x[{lo[0]:8.2f},{hi[0]:8.2f}] y[{lo[1]:7.2f},{hi[1]:7.2f}] "
              f"z[{lo[2]:8.2f},{hi[2]:8.2f}]  vol {m.volume:9,.0f}")
    tot = body.volume + flange.volume
    print(f"\n  분리 합계 {tot:,.0f} mm^3  vs 예전 통짜 {old.volume:,.0f} mm^3 "
          f"(차이 {tot - old.volume:+,.0f})")

    # 압착 간격 — 고정벽 면에서 각 압착면까지. full_workflow.py 의 LEFTWALL_GAP_*
    # 가 이 값을 물고 있으므로 여기서 찍어 두 곳이 어긋나지 않게 한다.
    ref_x = load(os.path.join(HERE, FIXED_WALL_REF + ".stl")).vertices.min(0)[0]
    g_flange = ref_x - flange.vertices.max(0)[0]
    g_body = ref_x - body.vertices.max(0)[0]
    print(f"\n  고정벽 면 x={ref_x:.2f} ({FIXED_WALL_REF}) 기준 중립(q=0) 압착 간격")
    print(f"    플랜지 면 x={flange.vertices.max(0)[0]:.2f} -> gap {g_flange:.2f} mm  (예전 12.00)")
    print(f"    본체 면   x={body.vertices.max(0)[0]:.2f} -> gap {g_body:.2f} mm  (예전 17.00)")
    print(f"  플랜지 면이 예전 대비 {flange.vertices.max(0)[0] - old.vertices.max(0)[0]:+.2f} mm "
          f"물러났다 — 새 CAD 의 차이지 변환 오차가 아니다(공통 부품 4개로 검증됨).")

    # 플랜지가 본체와 부피로 겹치면 IPC build 가 "Intersection detected" 로 죽는다
    # (§18 실측: 같은 body 안 형제 geom 이라도 겹치면 안 되고, 맞닿는 건 괜찮다).
    gap_y = flange.vertices.min(0)[1] - body.vertices.max(0)[1]
    print(f"  본체 상단 ~ 플랜지 하단 y 간격 {gap_y:+.3f} mm (음수면 겹침 = IPC 사망)")
    if gap_y < -1e-6:
        raise SystemExit("[중단] 본체와 플랜지가 부피로 겹친다.")

    print()
    print("=" * 74)
    print(f"[3/4] 본체 CoACD 볼록분해 (threshold={COACD_THRESHOLD})")
    print("=" * 74)
    hull_v = trimesh.convex.convex_hull(body).volume
    print(f"  분해 전: 실부피 {body.volume:,.0f} / hull {hull_v:,.0f} "
          f"-> hull 대비 {100*body.volume/hull_v:.1f}%  (이대로 두면 개구부가 막힌다)")
    parts = coacd.run_coacd(
        coacd.Mesh(body.vertices, body.faces),
        threshold=COACD_THRESHOLD,
        max_convex_hull=32,
        preprocess_mode="auto",
        mcts_iterations=150,
    )
    print(f"  hull {len(parts)}개 생성")

    # 이전 결과물을 남겨두면 XML 이 참조하는 개수와 어긋난다 — 먼저 지운다.
    for f in os.listdir(HERE):
        if f.startswith("L2_Left_Wall_body_hull_") and f.endswith(".stl"):
            os.remove(os.path.join(HERE, f))

    hull_tot, hull_ymax = 0.0, -1e9
    for i, (verts, faces) in enumerate(parts):
        h = trimesh.Trimesh(vertices=verts, faces=faces)
        h.export(os.path.join(HERE, OUT_BODY_HULL.format(i)))
        hull_tot += h.volume
        hull_ymax = max(hull_ymax, h.vertices.max(0)[1])
    # 시각 geom 은 hull 이 아니라 원형을 써야 한다 — hull 을 렌더하면 벽이
    # 뭉툭한 덩어리로 보인다. Genesis 는 group 3 충돌 geom 을 렌더하지 않고
    # (mjcf.py: group 0~2 만 시각으로 복제), 시각은 contype=0 geom 이 담당한다.
    body.export(os.path.join(HERE, OUT_BODY))
    flange.export(os.path.join(HERE, OUT_FLANGE))

    print(f"  hull 합계 부피 {hull_tot:,.0f} mm^3 = 본체 실부피의 "
          f"{100*hull_tot/body.volume:.1f}%")
    print(f"  hull 최고 y {hull_ymax:.3f} vs 플랜지 하단 {flange.vertices.min(0)[1]:.3f}")
    if hull_ymax > flange.vertices.min(0)[1] + 1e-6:
        raise SystemExit(
            "[중단] CoACD hull 이 플랜지 영역을 침범했다 — IPC build 가 죽는다. "
            "threshold 를 낮추거나 hull 을 잘라내야 한다."
        )

    print()
    print("=" * 74)
    print("[4/4] 산출물")
    print("=" * 74)
    print(f"  {OUT_BODY:34s} 본체 원형, 시각 전용 ({len(body.vertices)} verts)")
    print(f"  {OUT_FLANGE:34s} 플랜지, 시각+충돌 겸용 ({len(flange.vertices)} verts)")
    print(f"  {OUT_BODY_HULL.format(0)} ~ {OUT_BODY_HULL.format(len(parts)-1):>3s}   "
          f"본체 충돌, hull {len(parts)}개")
    print(f"\n  full_workflow.py 의 LEFTWALL_BODY_HULL_N 을 {len(parts)} 로 맞출 것.")


if __name__ == "__main__":
    sys.exit(main())

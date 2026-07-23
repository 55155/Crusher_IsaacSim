# -*- coding: utf-8 -*-
"""Fusion 360 API 전용 헬퍼 — joint anchor 추출 핵심 로직.

utills/dump_joint_anchors.py(Script)와 fusion_addin/dump_joint_anchors/(Add-In)
양쪽에서 공유하는 순수 로직. adsk.core/adsk.fusion 이 있어야 동작하므로 Fusion
360 안에서만 import 가능하다(Genesis/일반 파이썬 환경에서는 쓸 수 없음).
"""

import adsk.core, adsk.fusion
import math

CSV_COLUMNS = [
    "record_type",      # "anchor" 또는 "distance"
    "component",        # anchor: 대상 컴포넌트명 / distance: 대상 컴포넌트명
    "joint",            # anchor: joint 이름 / distance: "jointA <-> jointB"
    "occurrence_path",  # anchor 전용
    "tag",              # anchor 전용: "One" 또는 "Two"
    "other_component",  # anchor 전용: 상대편(joint의 반대쪽) 컴포넌트명
    "world_x_m", "world_y_m", "world_z_m",   # anchor 전용
    "local_x_m", "local_y_m", "local_z_m",   # anchor 전용 (MJCF anchor 후보)
    "distance_m",                            # distance 전용
    "anchor_vec_x_m", "anchor_vec_y_m", "anchor_vec_z_m",  # distance 전용
]


def _mat_to_local(occ, world_point):
    """월드 좌표의 점을 해당 occurrence(컴포넌트 인스턴스)의 로컬 좌표로 변환."""
    transform = occ.transform2.copy()
    ok = transform.invert()
    if not ok:
        return None
    p = world_point.copy()
    p.transformBy(transform)
    return p


def _iter_all_joints(comp, occ_path=""):
    for j in comp.joints:
        yield j, occ_path
    for occ in comp.occurrences:
        new_path = f"{occ_path}/{occ.name}" if occ_path else occ.name
        yield from _iter_all_joints(occ.component, new_path)


def collect_records(design, name_pattern: str):
    """
    name_pattern(부분 문자열, 대소문자 무시)에 이름이 매칭되는 모든 컴포넌트에
    대해 joint anchor 정보를 모아 CSV_COLUMNS 순서의 dict 리스트로 반환한다.
    좌표는 모두 m 단위(Fusion 내부 cm -> m 변환).

    name_pattern이 빈 문자열/공백("", "*", "all")이면 패턴 필터를 걸지 않고
    디자인 전체의 모든 joint/컴포넌트를 대상으로 한다 -- 컴포넌트별로 joint가
    2개 이상 물려있으면(폐루프 중복 export 케이스) 전부 자동으로 거리까지
    계산된다. 특정 부품만 보고 싶을 때만 패턴을 채워서 좁히면 된다.
    """
    root = design.rootComponent
    name_pattern_lower = name_pattern.strip().lower()
    match_all = name_pattern_lower in ("", "*", "all")

    all_occs = root.allOccurrences
    if match_all:
        target_occs = list(all_occs)
    else:
        target_occs = [occ for occ in all_occs if name_pattern_lower in occ.component.name.lower()]
    target_comp_names = {occ.component.name for occ in target_occs}

    all_joints = list(_iter_all_joints(root))

    records = []
    anchors_by_component = {}  # component name -> list of (joint_name, local_point[cm 그대로])

    for joint, path in all_joints:
        occ_one = joint.occurrenceOne
        occ_two = joint.occurrenceTwo
        comp_one_name = occ_one.component.name if occ_one else root.name
        comp_two_name = occ_two.component.name if occ_two else root.name

        if comp_one_name not in target_comp_names and comp_two_name not in target_comp_names:
            continue

        pairs = (
            ("One", joint.geometryOrOriginOne, occ_one, comp_one_name, comp_two_name),
            ("Two", joint.geometryOrOriginTwo, occ_two, comp_two_name, comp_one_name),
        )
        for tag, geo, occ, comp_name, other_name in pairs:
            try:
                world_pt = geo.origin
            except Exception:
                continue

            local_pt = None
            if comp_name in target_comp_names and occ is not None:
                local_pt = _mat_to_local(occ, world_pt)
                if local_pt is not None:
                    anchors_by_component.setdefault(comp_name, []).append((joint.name, local_pt))

            records.append({
                "record_type": "anchor",
                "component": comp_name,
                "joint": joint.name,
                "occurrence_path": path,
                "tag": tag,
                "other_component": other_name,
                "world_x_m": world_pt.x * 0.01,
                "world_y_m": world_pt.y * 0.01,
                "world_z_m": world_pt.z * 0.01,
                "local_x_m": local_pt.x * 0.01 if local_pt else "",
                "local_y_m": local_pt.y * 0.01 if local_pt else "",
                "local_z_m": local_pt.z * 0.01 if local_pt else "",
                "distance_m": "",
                "anchor_vec_x_m": "", "anchor_vec_y_m": "", "anchor_vec_z_m": "",
            })

    for comp_name, pts in anchors_by_component.items():
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                p1, p2 = pts[i][1], pts[j][1]
                dx, dy, dz = (p2.x - p1.x) * 0.01, (p2.y - p1.y) * 0.01, (p2.z - p1.z) * 0.01
                d = math.sqrt(dx * dx + dy * dy + dz * dz)
                records.append({
                    "record_type": "distance",
                    "component": comp_name,
                    "joint": f"{pts[i][0]} <-> {pts[j][0]}",
                    "occurrence_path": "",
                    "tag": "",
                    "other_component": "",
                    "world_x_m": "", "world_y_m": "", "world_z_m": "",
                    "local_x_m": "", "local_y_m": "", "local_z_m": "",
                    "distance_m": d,
                    "anchor_vec_x_m": dx, "anchor_vec_y_m": dy, "anchor_vec_z_m": dz,
                })

    return records, target_occs, len(all_joints)


def write_csv(records, filepath):
    import csv
    with open(filepath, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        w.writeheader()
        for r in records:
            w.writerow(r)


def build_report(design, name_pattern: str) -> str:
    """Text Commands 창에 보여줄 사람이 읽기 좋은 요약 문자열."""
    records, target_occs, n_joints = collect_records(design, name_pattern)

    lines = []
    lines.append(f"# Joint anchor dump — 대상 컴포넌트 이름에 '{name_pattern}' 포함")
    lines.append("# 좌표는 m 단위 (Fusion 내부 cm -> m 변환)")
    lines.append("")
    lines.append(f"매칭된 occurrence 수: {len(target_occs)}")
    for occ in target_occs:
        lines.append(f"  - {occ.fullPathName}  (component: {occ.component.name})")
    lines.append("")
    lines.append(f"디자인 전체 joint 수: {n_joints}")
    lines.append("")

    for r in records:
        if r["record_type"] == "anchor":
            lines.append(f"## joint={r['joint']}  tag={r['tag']}  component={r['component']}  "
                          f"(상대={r['other_component']})")
            lines.append(f"   world=({r['world_x_m']:.6f}, {r['world_y_m']:.6f}, {r['world_z_m']:.6f})")
            if r["local_x_m"] != "":
                lines.append(f"   local(anchor 후보)=({r['local_x_m']:.6f}, {r['local_y_m']:.6f}, {r['local_z_m']:.6f})")
        else:
            lines.append(f"## [{r['component']}] {r['joint']}  거리={r['distance_m']:.6f}m  "
                          f"anchor벡터=({r['anchor_vec_x_m']:.6f}, {r['anchor_vec_y_m']:.6f}, {r['anchor_vec_z_m']:.6f})")

    return "\n".join(lines)

"""
Crushing.usd 파일에서 링크(Link)와 조인트(Joint) 정보를 읽어 출력하고
USD_output.csv 로 저장하는 스크립트.

실행 방법:
  - Isaac Sim Python 환경: python ReadUSD.py
  - 일반 Python (pxr 설치 시): pip install usd-core, python ReadUSD.py
"""

import os
import sys
import math
import csv

# ── 경로 설정 ─────────────────────────────────────────────────────────────────
_HERE     = os.path.dirname(os.path.abspath(__file__))
USD_PATH  = os.path.join(_HERE, "..", "Crusher_IsaacSim_description", "Crushing.usd")
CSV_PATH  = os.path.join(_HERE, "USD_output.csv")

# ── pxr (OpenUSD) 임포트 ──────────────────────────────────────────────────────
try:
    from pxr import Usd, UsdGeom, UsdPhysics, Gf
    HAS_PXR = True
except ImportError:
    HAS_PXR = False
    print("[경고] pxr 라이브러리를 찾을 수 없습니다.")
    print("       Isaac Sim 환경이거나 'pip install usd-core' 로 설치하세요.")
    print("       URDF 파싱 폴백(fallback) 모드로 실행합니다.\n")


# ══════════════════════════════════════════════════════════════════════════════
# 공통 유틸
# ══════════════════════════════════════════════════════════════════════════════

def quat_to_rpy(q) -> tuple:
    """쿼터니언 → Roll-Pitch-Yaw (라디안) 변환."""
    x, y, z, w = q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2], q.GetReal()
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


def fmt_vec3(v) -> str:
    return f"({v[0]:+.6f}, {v[1]:+.6f}, {v[2]:+.6f})"


def fmt_rpy(roll, pitch, yaw) -> str:
    return (f"roll={math.degrees(roll):+.2f}°  "
            f"pitch={math.degrees(pitch):+.2f}°  "
            f"yaw={math.degrees(yaw):+.2f}°")


# ══════════════════════════════════════════════════════════════════════════════
# CSV 저장
# ══════════════════════════════════════════════════════════════════════════════

def save_csv(source_path: str,
             links_rows: list,
             joints_rows: list,
             joint_pos_rows: list,
             movable_rows: list):
    """
    수집된 데이터를 USD_output.csv 로 저장한다.

    각 섹션은 빈 행과 헤더 행으로 구분된다.
    """
    with open(CSV_PATH, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)

        # ── 메타 정보 ────────────────────────────────────────────────────────
        w.writerow(["source", source_path])
        w.writerow([])

        # ── 섹션 1: 링크 목록 ────────────────────────────────────────────────
        w.writerow(["[LINKS]", f"total={len(links_rows)}"])
        w.writerow(["#", "link_name", "world_pos_x [m]", "world_pos_y [m]", "world_pos_z [m]"])
        for row in links_rows:
            w.writerow(row)
        w.writerow([])

        # ── 섹션 2: 조인트 목록 ──────────────────────────────────────────────
        w.writerow(["[JOINTS]", f"total={len(joints_rows)}"])
        w.writerow(["#", "joint_name", "type", "parent (body0)", "child (body1)"])
        for row in joints_rows:
            w.writerow(row)
        w.writerow([])

        # ── 섹션 3: 조인트 접합 위치 ─────────────────────────────────────────
        w.writerow(["[JOINT_POSITIONS]"])
        w.writerow([
            "#", "joint_name", "type",
            "localPos0_x [m]", "localPos0_y [m]", "localPos0_z [m]",
            "localPos1_x [m]", "localPos1_y [m]", "localPos1_z [m]",
            "rot0_roll [deg]", "rot0_pitch [deg]", "rot0_yaw [deg]",
            "rot1_roll [deg]", "rot1_pitch [deg]", "rot1_yaw [deg]",
        ])
        for row in joint_pos_rows:
            w.writerow(row)
        w.writerow([])

        # ── 섹션 4: 가동 조인트 동역학 속성 ──────────────────────────────────
        w.writerow(["[MOVABLE_JOINTS]", f"total={len(movable_rows)}"])
        w.writerow([
            "joint_name", "type", "axis",
            "lower_limit", "upper_limit",
            "stiffness", "damping", "max_force [N]",
        ])
        for row in movable_rows:
            w.writerow(row)

    print(f"\n  → CSV 저장 완료: {CSV_PATH}")


# ══════════════════════════════════════════════════════════════════════════════
# 1. pxr 기반 USD 읽기
# ══════════════════════════════════════════════════════════════════════════════

def read_usd_with_pxr(usd_path: str):
    print("=" * 70)
    print(f"  USD 파일: {usd_path}")
    print("=" * 70)

    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print(f"[오류] USD 파일을 열 수 없습니다: {usd_path}")
        sys.exit(1)

    xform_cache = UsdGeom.XformCache()

    # ── 1-A. 링크 수집 ───────────────────────────────────────────────────────
    links = []
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            links.append(prim)

    links_rows = []   # CSV 용
    print(f"\n[링크 목록]  총 {len(links)}개\n")
    print(f"  {'#':>3}  {'링크 이름':<45}  {'월드 위치 (x, y, z) [m]'}")
    print("  " + "-" * 90)
    for i, prim in enumerate(links, 1):
        xf = xform_cache.GetLocalToWorldTransform(prim)
        t  = xf.ExtractTranslation()
        print(f"  {i:>3}  {prim.GetName():<45}  {fmt_vec3(t)}")
        links_rows.append([i, prim.GetName(), f"{t[0]:.6f}", f"{t[1]:.6f}", f"{t[2]:.6f}"])

    # ── 1-B. 조인트 수집 ─────────────────────────────────────────────────────
    joint_types = {
        "Fixed"     : UsdPhysics.FixedJoint,
        "Revolute"  : UsdPhysics.RevoluteJoint,
        "Prismatic" : UsdPhysics.PrismaticJoint,
        "Spherical" : UsdPhysics.SphericalJoint,
        "Distance"  : UsdPhysics.DistanceJoint,
    }

    joints = []
    for prim in stage.Traverse():
        for type_name, usd_type in joint_types.items():
            if prim.IsA(usd_type):
                joints.append((prim, type_name))
                break
        else:
            if prim.IsA(UsdPhysics.Joint):
                joints.append((prim, "Joint"))

    joints_rows = []   # CSV 용
    print(f"\n[조인트 목록]  총 {len(joints)}개\n")
    print(f"  {'#':>3}  {'조인트 이름':<45}  {'타입':<10}  "
          f"{'Body0 (parent)':<35}  {'Body1 (child)'}")
    print("  " + "-" * 140)

    for i, (prim, type_name) in enumerate(joints, 1):
        joint_api = UsdPhysics.Joint(prim)
        body0_rels = joint_api.GetBody0Rel().GetTargets()
        body1_rels = joint_api.GetBody1Rel().GetTargets()
        b0 = str(body0_rels[0]).split("/")[-1] if body0_rels else "—"
        b1 = str(body1_rels[0]).split("/")[-1] if body1_rels else "—"
        print(f"  {i:>3}  {prim.GetName():<45}  {type_name:<10}  {b0:<35}  {b1}")
        joints_rows.append([i, prim.GetName(), type_name, b0, b1])

    # ── 1-C. 조인트 접합 위치 ────────────────────────────────────────────────
    joint_pos_rows = []   # CSV 용
    print(f"\n[조인트 접합 위치 상세]\n")
    print(f"  {'#':>3}  {'조인트 이름':<45}  {'localPos0 (parent 기준)':<32}  "
          f"{'localPos1 (child 기준)'}")
    print("  " + "-" * 130)

    for i, (prim, type_name) in enumerate(joints, 1):
        pos0_attr = prim.GetAttribute("physics:localPos0")
        pos1_attr = prim.GetAttribute("physics:localPos1")
        pos0 = pos0_attr.Get() if pos0_attr and pos0_attr.IsValid() else None
        pos1 = pos1_attr.Get() if pos1_attr and pos1_attr.IsValid() else None

        s0 = fmt_vec3(pos0) if pos0 is not None else "N/A"
        s1 = fmt_vec3(pos1) if pos1 is not None else "N/A"
        print(f"  {i:>3}  {prim.GetName():<45}  {s0:<32}  {s1}")

        # 회전
        rot0_attr = prim.GetAttribute("physics:localRot0")
        rot1_attr = prim.GetAttribute("physics:localRot1")

        r0_deg = p0_deg = y0_deg = ""
        r1_deg = p1_deg = y1_deg = ""

        if rot0_attr and rot0_attr.IsValid():
            q0 = rot0_attr.Get()
            r0, p0_, y0 = quat_to_rpy(q0)
            r0_deg, p0_deg, y0_deg = math.degrees(r0), math.degrees(p0_), math.degrees(y0)
            print(f"  {'':>3}  {'  └ rot0 (RPY)':45}  {fmt_rpy(r0, p0_, y0)}")
        if rot1_attr and rot1_attr.IsValid():
            q1 = rot1_attr.Get()
            r1, p1_, y1 = quat_to_rpy(q1)
            r1_deg, p1_deg, y1_deg = math.degrees(r1), math.degrees(p1_), math.degrees(y1)
            print(f"  {'':>3}  {'  └ rot1 (RPY)':45}  {fmt_rpy(r1, p1_, y1)}")

        # CSV 행 (pos 단위: m, rot 단위: deg)
        def _v(vec, idx): return f"{vec[idx]:.6f}" if vec is not None else ""
        joint_pos_rows.append([
            i, prim.GetName(), type_name,
            _v(pos0, 0), _v(pos0, 1), _v(pos0, 2),
            _v(pos1, 0), _v(pos1, 1), _v(pos1, 2),
            f"{r0_deg:.4f}" if r0_deg != "" else "",
            f"{p0_deg:.4f}" if p0_deg != "" else "",
            f"{y0_deg:.4f}" if y0_deg != "" else "",
            f"{r1_deg:.4f}" if r1_deg != "" else "",
            f"{p1_deg:.4f}" if p1_deg != "" else "",
            f"{y1_deg:.4f}" if y1_deg != "" else "",
        ])

    # ── 1-D. 가동 조인트 동역학 속성 ─────────────────────────────────────────
    movable_rows = []   # CSV 용
    print(f"\n[가동 조인트 (non-fixed) 동역학 속성]\n")
    print(f"  {'조인트 이름':<45}  {'타입':<10}  {'축':<8}  "
          f"{'Lower Limit':>12}  {'Upper Limit':>12}  "
          f"{'Stiffness':>12}  {'Damping':>10}  {'MaxForce':>10}")
    print("  " + "-" * 130)

    for prim, type_name in joints:
        if type_name == "Fixed":
            continue

        axis_attr  = prim.GetAttribute("physics:axis")
        lower_attr = prim.GetAttribute("physics:lowerLimit")
        upper_attr = prim.GetAttribute("physics:upperLimit")

        axis  = axis_attr.Get()  if axis_attr  and axis_attr.IsValid()  else "—"
        lower = lower_attr.Get() if lower_attr and lower_attr.IsValid() else float("nan")
        upper = upper_attr.Get() if upper_attr and upper_attr.IsValid() else float("nan")

        stiffness = damping = max_force = float("nan")
        for drive_type in ("angular", "linear"):
            def _ga(name):
                a = prim.GetAttribute(f"drive:{drive_type}:physics:{name}")
                return a.Get() if a and a.IsValid() else float("nan")
            s = _ga("stiffness"); d = _ga("damping"); mf = _ga("maxForce")
            if not math.isnan(s):  stiffness = s
            if not math.isnan(d):  damping   = d
            if not math.isnan(mf): max_force = mf

        def _fs(v): return f"{v:.4f}" if not math.isnan(v) else "—"

        print(f"  {prim.GetName():<45}  {type_name:<10}  {str(axis):<8}  "
              f"{_fs(lower):>12}  {_fs(upper):>12}  "
              f"{_fs(stiffness):>12}  {_fs(damping):>10}  {_fs(max_force):>10}")

        movable_rows.append([
            prim.GetName(), type_name, str(axis),
            _fs(lower), _fs(upper),
            _fs(stiffness), _fs(damping), _fs(max_force),
        ])

    print("\n" + "=" * 70)
    print("  읽기 완료")
    print("=" * 70)

    # ── CSV 저장 ──────────────────────────────────────────────────────────────
    save_csv(usd_path, links_rows, joints_rows, joint_pos_rows, movable_rows)


# ══════════════════════════════════════════════════════════════════════════════
# 2. URDF 기반 폴백 (pxr 없을 때)
# ══════════════════════════════════════════════════════════════════════════════

def read_urdf_fallback():
    """pxr가 없을 때 URDF 파일을 파싱하여 동일한 정보를 출력하고 CSV 로 저장한다."""
    import xml.etree.ElementTree as ET

    urdf_path = os.path.join(_HERE, "..", "Crusher_IsaacSim_description",
                             "urdf", "Crusher_IsaacSim.urdf")

    if not os.path.exists(urdf_path):
        print(f"[오류] URDF 파일도 찾을 수 없습니다: {urdf_path}")
        sys.exit(1)

    print("=" * 70)
    print(f"  URDF 파일 (폴백): {urdf_path}")
    print("  ※ Crushing.usd 의 링크·조인트 구조는 URDF 와 동일합니다.")
    print("=" * 70)

    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # ── 링크 ─────────────────────────────────────────────────────────────────
    links = root.findall("link")
    links_rows = []
    print(f"\n[링크 목록]  총 {len(links)}개\n")
    print(f"  {'#':>3}  {'링크 이름':<45}  {'질량 [kg]':>12}  {'관성 원점 (x, y, z)'}")
    print("  " + "-" * 100)
    for i, link in enumerate(links, 1):
        name     = link.get("name", "?")
        inertial = link.find("inertial")
        if inertial is not None:
            mass_el = inertial.find("mass")
            orig_el = inertial.find("origin")
            mass = float(mass_el.get("value", 0)) if mass_el is not None else 0.0
            xyz  = orig_el.get("xyz", "0 0 0") if orig_el is not None else "0 0 0"
            vals = [float(v) for v in xyz.split()]
            pos_s = f"({vals[0]:+.4f}, {vals[1]:+.4f}, {vals[2]:+.4f})"
            print(f"  {i:>3}  {name:<45}  {mass:>12.5f}  {pos_s}")
            links_rows.append([i, name, f"{vals[0]:.6f}", f"{vals[1]:.6f}", f"{vals[2]:.6f}"])
        else:
            print(f"  {i:>3}  {name:<45}  {'—':>12}  —")
            links_rows.append([i, name, "", "", ""])

    # ── 조인트 ───────────────────────────────────────────────────────────────
    joints = root.findall("joint")
    joints_rows     = []
    joint_pos_rows  = []
    movable_rows    = []

    print(f"\n[조인트 목록]  총 {len(joints)}개\n")
    print(f"  {'#':>3}  {'조인트 이름':<45}  {'타입':<12}  "
          f"{'부모(parent)':<30}  {'자식(child)'}")
    print("  " + "-" * 130)
    for i, joint in enumerate(joints, 1):
        name   = joint.get("name", "?")
        jtype  = joint.get("type", "?")
        parent = joint.find("parent")
        child  = joint.find("child")
        p_name = parent.get("link", "?") if parent is not None else "?"
        c_name = child.get("link",  "?") if child  is not None else "?"
        print(f"  {i:>3}  {name:<45}  {jtype:<12}  {p_name:<30}  {c_name}")
        joints_rows.append([i, name, jtype, p_name, c_name])

    # ── 조인트 접합 위치 ─────────────────────────────────────────────────────
    print(f"\n[조인트 접합 위치 (parent 좌표계 기준)]\n")
    print(f"  {'#':>3}  {'조인트 이름':<45}  {'타입':<12}  "
          f"{'xyz [m]':<36}  {'rpy [rad]'}")
    print("  " + "-" * 130)
    for i, joint in enumerate(joints, 1):
        name  = joint.get("name", "?")
        jtype = joint.get("type", "?")
        orig  = joint.find("origin")
        if orig is not None:
            xyz_v = [float(v) for v in orig.get("xyz", "0 0 0").split()]
            rpy_v = [float(v) for v in orig.get("rpy", "0 0 0").split()]
            xyz_fmt = f"({xyz_v[0]:+.5f}, {xyz_v[1]:+.5f}, {xyz_v[2]:+.5f})"
            rpy_fmt = f"({rpy_v[0]:+.5f}, {rpy_v[1]:+.5f}, {rpy_v[2]:+.5f})"
            print(f"  {i:>3}  {name:<45}  {jtype:<12}  {xyz_fmt:<36}  {rpy_fmt}")
            joint_pos_rows.append([
                i, name, jtype,
                f"{xyz_v[0]:.6f}", f"{xyz_v[1]:.6f}", f"{xyz_v[2]:.6f}",
                "", "", "",   # localPos1 없음
                f"{math.degrees(rpy_v[0]):.4f}",
                f"{math.degrees(rpy_v[1]):.4f}",
                f"{math.degrees(rpy_v[2]):.4f}",
                "", "", "",
            ])
        else:
            print(f"  {i:>3}  {name:<45}  {jtype:<12}  —")
            joint_pos_rows.append([i, name, jtype, "", "", "", "", "", "", "", "", "", "", "", ""])

    # ── 가동 조인트 ──────────────────────────────────────────────────────────
    movable_types = {"revolute", "continuous", "prismatic"}
    movable = [j for j in joints if j.get("type", "") in movable_types]

    if movable:
        print(f"\n[가동 조인트 축 및 범위]\n")
        print(f"  {'조인트 이름':<45}  {'타입':<12}  {'축 (xyz)':<20}  "
              f"{'하한 [rad/m]':>14}  {'상한 [rad/m]':>14}")
        print("  " + "-" * 115)
        for joint in movable:
            name     = joint.get("name", "?")
            jtype    = joint.get("type", "?")
            axis_el  = joint.find("axis")
            limit_el = joint.find("limit")
            axis_xyz = axis_el.get("xyz", "1 0 0") if axis_el is not None else "1 0 0"
            lower = upper = "—"
            if limit_el is not None:
                lower = limit_el.get("lower", "—")
                upper = limit_el.get("upper", "—")
            try:    l_s = f"{float(lower):+.5f}"
            except: l_s = lower
            try:    u_s = f"{float(upper):+.5f}"
            except: u_s = upper
            print(f"  {name:<45}  {jtype:<12}  {axis_xyz:<20}  {l_s:>14}  {u_s:>14}")
            movable_rows.append([name, jtype, axis_xyz, l_s, u_s, "—", "—", "—"])

    print("\n" + "=" * 70)
    print("  읽기 완료")
    print("=" * 70)

    save_csv(urdf_path, links_rows, joints_rows, joint_pos_rows, movable_rows)


# ══════════════════════════════════════════════════════════════════════════════
# 진입점
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    if HAS_PXR:
        if not os.path.exists(USD_PATH):
            print(f"[오류] USD 파일을 찾을 수 없습니다: {USD_PATH}")
            sys.exit(1)
        read_usd_with_pxr(USD_PATH)
    else:
        read_urdf_fallback()

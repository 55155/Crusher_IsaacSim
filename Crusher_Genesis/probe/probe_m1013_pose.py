"""probe_m1013_pose.py — M0609 -> M1013 교체 + 받침 지그(140mm)에 맞춰
`Q_GRASP`/`Q_LIFT` 를 다시 푼다 (2026-10-14).

왜 다시 풀어야 하나: 두 상수는 **M0609 기구학으로 푼 관절각**이다(full_workflow
§Q_GRASP 주석: 2026-07-24 에 위치는 유지하고 orientation 만 수직으로 강제해 IK 로
재계산한 값). 링크 길이가 0.411/0.368 -> 0.62/0.559m 로 바뀌고 베이스가 지그만큼
올라가면 같은 관절각은 전혀 다른 곳을 가리킨다.

무엇을 보존하나: **TCP 의 월드 자세**. 봉투 스폰(BAG_POS), 선반(SHELF_POS), 정제
낙하(TABLET_POS)가 전부 FINGER_MID 에서 파생되고, 슬롯·회수장치·압착은 월드
좌표에 고정돼 있다. 그러므로 교체 전 M0609 가 두 자세에서 만들던 TCP 월드 좌표를
목표로 M1013 기구학으로 IK 를 다시 풀면, 공정 전체가 같은 월드 좌표를 유지한다
(변인통제: 바뀐 것은 팔과 그 밑 지그뿐).

절차
  1. MuJoCo FK — 구 에셋(m0609_rg2_v2.xml) + 구 오프셋(-0.225, 0.25, 0.0) 에서
     구 Q_GRASP/Q_LIFT 의 TCP 월드 좌표를 읽는다. (probe_waypoints.py 가 쓰는
     것과 같은 FK 경로: f1 링크 원점 + xmat @ FINGER_TCP_LOCAL)
  2. Genesis IK — full_workflow 가 실제로 쓰는 패치본 MJCF(M1013)를 그 코드로
     굽고(`F._prepare_robot_mjcf()`), 핑거를 FING_CLOSE 로 고정한 뒤 1의 좌표와
     VERTICAL_QUAT(수직, 손목 트위스트 적용 전)로 arm 6DOF 를 푼다.
  3. 검산 — 새 관절각의 FK TCP 가 목표와 얼마나 맞는지, 그리고 FINGER_MID_BASE
     (= 목표 - 새 오프셋)가 full_workflow 상수와 일치하는지 출력한다.

사용법:  python probe_m1013_pose.py
"""
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE),
                                "Crusher_M0609_RG2_Tablet_Samplebag"))
import full_workflow as F                                    # noqa: E402

# ── 교체 전(M0609) 기준값 — full_workflow 에서 지워졌으므로 여기에 박아 둔다 ──
M0609_XML = os.path.join(F.paths.ROBOTS_DIR, "m0609_rg2_v2.xml")
M0609_OFFSET = np.array([-0.2250, 0.2500, 0.0])
Q_GRASP_M0609 = np.array([-0.00063, -0.23137, 1.17360, 0.00050, 2.19917, -0.00054])
Q_LIFT_M0609 = np.array([-0.00063, 0.11974, 0.31246, 0.00097, 2.70918, 0.00005])
VERTICAL_Q = np.array([0.0, 0.0, 1.0, 0.0])   # 손목 트위스트 적용 전(상수 원본)


def mj_tcp(xml, q_arm, fing):
    """MJCF + arm 6DOF + 핑거값으로 TCP(f1-f2 진짜 중앙)의 base 프레임 좌표."""
    import mujoco
    m = mujoco.MjModel.from_xml_path(xml)
    d = mujoco.MjData(m)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, F.FINGER_LINKS[0])
    d.qpos[:6] = q_arm
    d.qpos[6:12] = fing
    mujoco.mj_forward(m, d)
    R = d.xmat[bid].reshape(3, 3)
    return d.xpos[bid] + R @ F.FINGER_TCP_LOCAL, R


def main():
    mm = 1e3
    tcp_grasp, _ = mj_tcp(M0609_XML, Q_GRASP_M0609, F.FING_CLOSE)
    tcp_lift, _ = mj_tcp(M0609_XML, Q_LIFT_M0609, F.FING_CLOSE)
    tgt_grasp = tcp_grasp + M0609_OFFSET
    tgt_lift = tcp_lift + M0609_OFFSET
    print("[M0609] 보존할 TCP 월드 좌표 (mm)")
    print(f"  grasp  {np.round(tgt_grasp * mm, 3)}")
    print(f"  lift   {np.round(tgt_lift * mm, 3)}")
    print(f"  grasp - 구오프셋 = {np.round(tcp_grasp, 8)}   "
          f"(코드 FINGER_MID_BASE 구값 [0.20346321 0.0061799 0.43625346])")

    print(f"\n[M1013] 새 오프셋 {F.ROBOT_OFFSET.tolist()}  "
          f"(지그 {F.JIG_SIZE[2]*mm:.0f}mm)")
    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning")
    scene = gs.Scene(show_viewer=False)
    robot = scene.add_entity(
        gs.morphs.MJCF(file=F._prepare_robot_mjcf(), pos=tuple(F.ROBOT_OFFSET),
                       decimate=False),
    )
    scene.build()
    link = robot.get_link(F.FINGER_LINKS[0])

    out = {}
    for name, tgt, q0 in (("grasp", tgt_grasp, Q_GRASP_M0609),
                          ("lift", tgt_lift, Q_LIFT_M0609)):
        # 핑거는 FING_CLOSE 로 고정하고(TCP 기준과 같은 상태) arm 만 푼다.
        robot.set_dofs_position(np.concatenate([q0, [F.FING_CLOSE] * 6]))
        q = np.asarray(robot.inverse_kinematics(
            link=link, pos=tgt, quat=VERTICAL_Q,
            local_point=F.FINGER_TCP_LOCAL,
            dofs_idx_local=np.arange(6)).cpu()).squeeze()[:6]
        robot.set_dofs_position(np.concatenate([q, [F.FING_CLOSE] * 6]))
        got = np.asarray(link.get_pos().cpu()).squeeze()
        quat = np.asarray(link.get_quat().cpu()).squeeze()
        R = _quat_to_R(quat)
        got_tcp = got + R @ F.FINGER_TCP_LOCAL
        err = (got_tcp - tgt) * mm
        out[name] = q
        print(f"\n  {name}: q = [{', '.join(f'{v:.5f}' for v in q)}]")
        print(f"    FK TCP {np.round(got_tcp * mm, 3)}   오차 {np.round(err, 3)} mm "
              f"(|e| {np.linalg.norm(err):.3f} mm)")
        print(f"    quat   {np.round(quat, 5)}  (목표 {VERTICAL_Q})")
        print(f"    관절 한계 여유: joint_3 |{q[2]:+.3f}| vs 2.7925")

    # MuJoCo FK 로 교차검증 — Genesis IK 해를 같은 MJCF 에 넣어 다시 본다.
    print("\n[교차검증] 같은 해를 MuJoCo FK 로 (Genesis 와 1e-5 m 안이면 동일 기구학)")
    for name, tgt in (("grasp", tgt_grasp), ("lift", tgt_lift)):
        p, _ = mj_tcp(F.ROBOT_MJCF, out[name], F.FING_CLOSE)
        print(f"  {name}: {np.round((p + F.ROBOT_OFFSET) * mm, 3)}  "
              f"오차 {np.round((p + F.ROBOT_OFFSET - tgt) * mm, 3)} mm")

    print("\n[결과] full_workflow.py 에 넣을 값")
    for name in ("grasp", "lift"):
        q = out[name]
        print(f"  Q_{name.upper()} = np.array([{', '.join(f'{v:.5f}' for v in q)}], float)")
    fmb = tgt_grasp - F.ROBOT_OFFSET
    print(f"  FINGER_MID_BASE = {np.round(fmb, 8).tolist()}")
    print(f"  (코드 상수 {np.round(F.FINGER_MID_BASE, 8).tolist()} 와 차이 "
          f"{np.round((fmb - F.FINGER_MID_BASE) * mm, 4)} mm)")


def _quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


if __name__ == "__main__":
    main()

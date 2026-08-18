# -*- coding: utf-8 -*-
"""
mujoco_slow_torque_test.py — 회수장치2를 순수 MuJoCo 물리로(Genesis 아님)
아주 느린 속도/힘으로 구동해보는 테스트.

지금까지 Genesis 쪽 테스트들은 매 스텝 set_dofs_position으로 메인 샤프트
조인트를 강제로 원하는 각도에 꽂아넣는 "키네마틱 구동"이었다(그래서
connect 제약의 실제 잔차/거동을 정확히 보려면 mujoco 자체 솔버로 따로
검증해야 했음). 이 스크립트는 그 대신 실제 motor actuator에 아주 작은
토크를 걸어서, 관성 + connect 제약 + (테스트를 위해 추가한) 조인트 댐핑이
자연스럽게 반응하도록 만든다 -- mujoco 자체 예제인
genesis/assets/xml/four_bar_linkage.xml 이 <motor>로 구동하는 것과 동일한
방식.

**에셋 교체(2026-08-19)**: recovery2_mjcf/recovery2.xml(fusion2xml 직행 빌드).
조인트/액추에이터 이름이 한글 원본으로 바뀌었고(회전_31 / 회전_31_act /
슬라이더_35), 폐루프 connect 가 실제 바디끼리(Crank_1 ↔ M_Top_1)라 잔차는
컴파일된 eq_data 의 anchor 두 개를 그대로 읽어서 잰다.

damping: 구 XML 에는 없어서 이 스크립트가 직접 걸어줬는데, 신 XML 은
<default><joint damping="0.1"> 로 세 조인트 모두에 이미 걸려 있다 — 아래 대입은
따라서 "없던 감쇠를 추가"가 아니라 샤프트만 0.1 -> DAMPING 으로 **낮추는**
동작이다(원본 XML 은 여전히 안 건드린다).

사용법:
    python mujoco_slow_torque_test.py            # mp4로 저장
    VIEWER=1 python mujoco_slow_torque_test.py    # 인터랙티브 뷰어로 직접 확인
"""
import os, sys
from datetime import datetime
import numpy as np
import mujoco

_DIR = os.path.dirname(os.path.abspath(__file__))
XML = os.path.normpath(os.path.join(
    _DIR, "..", "assets", "robots", "recovery2_mjcf", "recovery2.xml"))

SHAFT_JOINT = "회전_31"        # 구 빌드의 base_link_Shaft_copy_1
SHAFT_MOTOR = "회전_31_act"    # 구 빌드의 base_link_Shaft_copy_1_motor
SLIDE_JOINT = "슬라이더_35"     # 구 빌드의 base_link_M_Bottom_1
LOOP_EQUALITY = "connect_Crank_1_M_Top_1"

DT = 1.0e-3
DAMPING = 0.05     # N*m*s/rad, 샤프트 joint에만 임시로 부여 (원본 XML에는 없음)
TORQUE = 0.003     # N*m, 아주 작은 상수 토크 -> 종단속도 약 3~4 deg/s
SIM_SECONDS = 40.0
N_STEPS = int(SIM_SECONDS / DT)

OUT_DIR = os.path.join(_DIR, "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)


def build_model():
    os.chdir(os.path.dirname(XML))
    model = mujoco.MjModel.from_xml_path(XML)
    model.opt.timestep = DT

    shaft_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, SHAFT_JOINT)
    dof_adr = model.jnt_dofadr[shaft_jid]
    model.dof_damping[dof_adr] = DAMPING

    data = mujoco.MjData(model)
    return model, data, shaft_jid


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" 회수장치2 -- MuJoCo 저속/저토크 구동 테스트 (viewer={use_viewer})")
    print(f" torque={TORQUE} N*m, shaft damping={DAMPING} (테스트 전용, XML 미변경)")
    print("=" * 60)

    model, data, shaft_jid = build_model()
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, SHAFT_MOTOR)
    slide_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, SLIDE_JOINT)
    # 폐루프 잔차: 컴파일된 eq_data 에 anchor 가 body1/body2 로컬로 각각 들어있다
    # (data[0:3]=body1 로컬, data[3:6]=body2 로컬). 구 빌드는 두 바디가 기준자세
    # 에서 겹쳐 있어 같은 anchor 를 두 번 써도 됐지만 신 빌드는 그렇지 않다.
    eq_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, LOOP_EQUALITY)
    body1_id, body2_id = int(model.eq_obj1id[eq_id]), int(model.eq_obj2id[eq_id])
    anchor1 = model.eq_data[eq_id][:3].copy()
    anchor2 = model.eq_data[eq_id][3:6].copy()

    data.ctrl[act_id] = TORQUE

    def loop_err():
        far1 = data.xpos[body1_id] + data.xmat[body1_id].reshape(3, 3) @ anchor1
        far2 = data.xpos[body2_id] + data.xmat[body2_id].reshape(3, 3) @ anchor2
        return np.linalg.norm(far1 - far2)

    shaft_qadr = model.jnt_qposadr[shaft_jid]
    shaft_dadr = model.jnt_dofadr[shaft_jid]
    slide_qadr = model.jnt_qposadr[slide_jid]

    def log(k):
        deg = np.degrees(data.qpos[shaft_qadr])
        deg_s = np.degrees(data.qvel[shaft_dadr])
        slide_mm = data.qpos[slide_qadr] * 1e3
        err_mm = loop_err() * 1e3
        print(f"  t={k*DT:6.2f}s  shaft={deg:7.2f}deg ({deg_s:5.2f}deg/s)  "
              f"slider={slide_mm:+7.2f}mm  loop_err={err_mm:.3f}mm")

    if use_viewer:
        from mujoco import viewer as mj_viewer
        with mj_viewer.launch_passive(model, data) as viewer:
            k = 0
            while viewer.is_running() and k < N_STEPS:
                mujoco.mj_step(model, data)
                viewer.sync()
                if k % 2000 == 0:
                    log(k)
                k += 1
    else:
        import cv2
        model.vis.global_.offwidth = 1280
        model.vis.global_.offheight = 960
        renderer = mujoco.Renderer(model, height=960, width=1280)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [-0.05, 0.06, 0.06]
        cam.distance = 0.35
        cam.azimuth = 135
        cam.elevation = -25

        _TS = datetime.now().strftime("%Y%m%d_%H%M%S")
        MP4 = os.path.join(OUT_DIR, f"mujoco_slow_torque_{_TS}.mp4")
        vw = cv2.VideoWriter(MP4, cv2.VideoWriter_fourcc(*"mp4v"), 30, (1280, 960))

        render_every = max(1, int(round(1.0 / (30 * DT))))  # 30fps에 맞춰 다운샘플
        for k in range(N_STEPS):
            mujoco.mj_step(model, data)
            if k % render_every == 0:
                renderer.update_scene(data, camera=cam)
                frame = renderer.render()
                vw.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            if k % 2000 == 0:
                log(k)

        vw.release()
        renderer.close()
        print(f"\n[saved] {MP4}")

    print("완료.")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")

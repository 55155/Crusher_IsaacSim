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

damping은 XML 파일 자체에는 없으므로(추가하면 다른 용도의 시뮬레이션에도
영향을 주게 됨) 이 스크립트에서 로드 후 model.dof_damping[]으로만 임시로
걸어준다.

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
    _DIR, "..", "assets", "robots", "회수장치2_description", "회수장치2.xml"))

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

    shaft_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_link_Shaft_copy_1")
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
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_link_Shaft_copy_1_motor")
    slide_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_link_M_Bottom_1")
    crank_b_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Crank_1_b")
    crank_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Crank_1")
    anchor_local = np.array([0.06, 0.0, 0.006])

    data.ctrl[act_id] = TORQUE

    def loop_err():
        p1 = data.xpos[crank_b_id]
        R1 = data.xmat[crank_b_id].reshape(3, 3)
        p2 = data.xpos[crank_id]
        R2 = data.xmat[crank_id].reshape(3, 3)
        far1 = p1 + R1 @ anchor_local
        far2 = p2 + R2 @ anchor_local
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

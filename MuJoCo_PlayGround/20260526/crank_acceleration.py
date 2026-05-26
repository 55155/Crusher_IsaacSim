"""
crank_acceleration.py
크랭크-슬라이더 메커니즘의 가속도 측정 및 플롯

측정 항목:
  ─ 크랭크 (L4_Shaft_1 / joint: L3_Bevel_GearBox_1_L4_Shaft_1)
      각도 [deg] / 각속도 [rad/s] / 각가속도 [rad/s²]
  ─ 슬라이더 (L8_Link3_Shaft_1 / joint: L2_Linear_bush_1_L8_Link3_Shaft_1)
      위치 [mm]  / 속도 [mm/s]   / 가속도 [mm/s²]
      (qacc 기준값 + xacc world-Y 성분 비교)

실행:
    conda activate isaac_sim
    python crank_acceleration.py

뷰어가 열리면 시뮬레이션이 SIM_DURATION 초 동안 진행됩니다.
시간이 끝나거나 뷰어를 닫으면 matplotlib 플롯이 표시됩니다.
플롯은 같은 폴더에 crank_acceleration.png 로 자동 저장됩니다.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer

# ── 경로 ─────────────────────────────────────────────────────────────
_HERE     = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.normpath(os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml"))

# ── 파라미터 ─────────────────────────────────────────────────────────
SIM_DURATION = 5.0    # 측정 시간 [s]
MOTOR_CTRL   = 10.0   # Motor1_crank 제어 입력 [N·m]

# ── 모델 로드 ─────────────────────────────────────────────────────────
model = mujoco.MjModel.from_xml_path(MJCF_PATH)
data  = mujoco.MjData(model)

# ── ID 조회 ──────────────────────────────────────────────────────────
def _jnt(name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)

def _body(name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)

def _act(name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

j_crank  = _jnt("L3_Bevel_GearBox_1_L4_Shaft_1")
j_slider = _jnt("L2_Linear_bush_1_L8_Link3_Shaft_1")
b_crank  = _body("L4_Shaft_1")
b_slider = _body("L8_Link3_Shaft_1")
act_id   = _act("Motor1_crank")

# qpos / qvel / qacc 배열 인덱스
qa_crank  = model.jnt_qposadr[j_crank]   # qpos index (hinge)
da_crank  = model.jnt_dofadr[j_crank]    # qvel/qacc index
qa_slider = model.jnt_qposadr[j_slider]  # qpos index (slide)
da_slider = model.jnt_dofadr[j_slider]

print("=" * 55)
print("  Crank-Slider Acceleration Measurement")
print(f"  모터 제어 입력 : {MOTOR_CTRL} N·m")
print(f"  측정 시간      : {SIM_DURATION} s")
print(f"  크랭크 joint id: {j_crank}  (dofadr={da_crank})")
print(f"  슬라이더 jnt id: {j_slider}  (dofadr={da_slider})")
print("=" * 55)

# ── 데이터 버퍼 ──────────────────────────────────────────────────────
t_log      = []
crank_ang  = []   # qpos [rad]
crank_vel  = []   # qvel [rad/s]
crank_acc  = []   # qacc [rad/s²]
slide_pos  = []   # qpos [m]
slide_vel  = []   # qvel [m/s]
slide_acc  = []   # qacc [m/s²]
xacc_buf   = []   # xacc[b_slider] 6D spatial acc

# ── 시뮬레이션 + 데이터 수집 ─────────────────────────────────────────
data.ctrl[act_id] = MOTOR_CTRL

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = False
    viewer.opt.geomgroup[3] = False

    print("뷰어 실행 중... (시간이 끝나거나 뷰어를 닫으면 플롯이 표시됩니다)")

    while viewer.is_running() and data.time < SIM_DURATION:
        mujoco.mj_step(model, data)

        t_log.append(data.time)
        crank_ang.append(data.qpos[qa_crank])
        crank_vel.append(data.qvel[da_crank])
        crank_acc.append(data.qacc[da_crank])
        slide_pos.append(data.qpos[qa_slider])
        slide_vel.append(data.qvel[da_slider])
        slide_acc.append(data.qacc[da_slider])
        # cacc: com-based spatial acc in world frame → [ang_acc(3), lin_acc(3)]
        # (Python 바인딩에서 xacc 대신 cacc 사용)
        xacc_buf.append(data.cacc[b_slider].copy())

        viewer.sync()

# ── numpy 변환 ───────────────────────────────────────────────────────
t  = np.array(t_log)
ca = np.degrees(np.array(crank_ang))   # deg
cv = np.array(crank_vel)               # rad/s
cA = np.array(crank_acc)               # rad/s²
sp = np.array(slide_pos)  * 1e3        # mm
sv = np.array(slide_vel)  * 1e3        # mm/s
sA = np.array(slide_acc)  * 1e3        # mm/s²
xa = np.array(xacc_buf)                # (N, 6), world frame

# 슬라이더 world 선가속도: xacc[3:6] = linear acc
# 슬라이더 slide 방향 = joint "axis=1 0 0" + body quat(0.5,0.5,0.5,0.5)
# → body local X = world Y → xa[:, 4] 가 주 운동 방향
xa_y = xa[:, 4] * 1e3   # mm/s²  (world Y 성분)

n = len(t)
if n == 0:
    print("[경고] 수집된 데이터 없음. 뷰어가 너무 빨리 닫혔습니다.")
    import sys; sys.exit(0)

print(f"\n수집 완료: {n} 스텝  ({t[-1]:.3f} s)")
print(f"  크랭크 각도   범위: {ca.min():.1f} ~ {ca.max():.1f} deg")
print(f"  크랭크 각가속 범위: {cA.min():.2f} ~ {cA.max():.2f} rad/s²")
print(f"  슬라이더 위치 범위: {sp.min():.2f} ~ {sp.max():.2f} mm")
print(f"  슬라이더 가속 범위: {sA.min():.2f} ~ {sA.max():.2f} mm/s²")

# ── 플롯 ─────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=True)
fig.suptitle(
    f"Crank-Slider Mechanism — Acceleration Analysis\n"
    f"Motor={MOTOR_CTRL} N·m  |  dt={model.opt.timestep*1e3:.1f} ms  |  t={t[-1]:.2f} s",
    fontsize=12, fontweight="bold"
)

# ── 왼쪽 열: 크랭크 ──────────────────────────────────────────────────
axes[0, 0].plot(t, ca, color="tab:blue")
axes[0, 0].set_title("크랭크 각도 (Crank Angle)")
axes[0, 0].set_ylabel("θ [deg]")

axes[1, 0].plot(t, cv, color="tab:orange")
axes[1, 0].set_title("크랭크 각속도 (Angular Velocity)")
axes[1, 0].set_ylabel("ω [rad/s]")

axes[2, 0].plot(t, cA, color="tab:red")
axes[2, 0].set_title("크랭크 각가속도 (Angular Acceleration)")
axes[2, 0].set_ylabel("α [rad/s²]")
axes[2, 0].set_xlabel("Time [s]")

# ── 오른쪽 열: 슬라이더 ──────────────────────────────────────────────
axes[0, 1].plot(t, sp, color="tab:green")
axes[0, 1].set_title("슬라이더 위치 (Slider Position)")
axes[0, 1].set_ylabel("x [mm]")

axes[1, 1].plot(t, sv, color="tab:purple")
axes[1, 1].set_title("슬라이더 속도 (Slider Velocity)")
axes[1, 1].set_ylabel("v [mm/s]")

axes[2, 1].plot(t, sA,   color="tab:brown", label="qacc (joint DOF)")
axes[2, 1].plot(t, xa_y, color="tab:cyan",  linestyle="--", alpha=0.75,
                label="xacc world-Y (body COM)")
axes[2, 1].set_title("슬라이더 선가속도 (Linear Acceleration)")
axes[2, 1].set_ylabel("a [mm/s²]")
axes[2, 1].set_xlabel("Time [s]")
axes[2, 1].legend(fontsize=8, loc="upper right")

for ax in axes.flat:
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="k", linewidth=0.5)

plt.tight_layout()

save_path = os.path.join(_HERE, "crank_acceleration.png")
plt.savefig(save_path, dpi=150, bbox_inches="tight")
print(f"\n플롯 저장 완료: {save_path}")
plt.show()

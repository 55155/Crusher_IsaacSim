"""
crusher_tablet_slidejoint.py  [v1 — slide joint, Y축 자유 이동]
Crusher + Tablet 통합 시뮬레이션

▶ 알약 고정 방식: slide joint (Y축)
    - mocap 고정 대신 알약이 Y축 방향으로 자유롭게 밀려남
    - 슬라이더가 밀면 알약이 뒷벽(L1_Wall1_1)에 눌리며 압축
    - 반력은 mj_contactForce 로 수집

    [비교 목적]
    mocap 방식 (crusher_tablet_sim.py v7) vs slide joint 방식 (본 파일)
    → 알약이 고정일 때 vs 실제로 눌릴 때의 반력 프로파일 차이 확인

▶ Phase 1  (PHASE1_STEPS 스텝, 뷰어 없음)
    lock_crank equality 활성 (-90°) → 메커니즘 안정화

▶ Phase 2  (뷰어 오픈)
    MOTOR_DELAY 초 후 lock_crank 해제 → Motor CCW 구동
    알약은 Y축 slide joint 로 자유롭게 이동 (뒷벽에 밀착)
    Moving-window stall 감지: 크랭크 속도 < STALL_VEL_THR × STALL_TIME_S → 방향 전환

▶ 배치 좌표 (MuJoCo world frame)
    PLACE_X_MM = -47.879  →  MuJoCo X
    PLACE_Z_MM =  50.108  →  MuJoCo Z
    WALL_Y_MM  = 336.199  →  MuJoCo Y  (충돌판 벽 표면)
    알약 초기 중심 Y = WALL_Y_MM - half_th  (표면이 벽에 접촉)

▶ 실행
    conda activate isaac_sim
    python crusher_tablet_slidejoint.py [tablet.stl]
    python crusher_tablet_slidejoint.py [tablet.stl] --density 1400
    python crusher_tablet_slidejoint.py [tablet.stl] --damping 50
"""

import os
import sys
import re
import csv
import math
import argparse
import xml.etree.ElementTree as ET
from collections import deque
from datetime import datetime

import numpy as np
try:
    _np_trapz = np.trapezoid
except AttributeError:
    _np_trapz = np.trapz

import matplotlib
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'Malgun Gothic'
plt.rcParams['axes.unicode_minus'] = False
import mujoco
import mujoco.viewer

# ── 경로 ─────────────────────────────────────────────────────────────
_HERE     = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.normpath(
    os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml"))
MJCF_DIR  = os.path.dirname(MJCF_PATH)
STL_DIR   = os.path.normpath(
    os.path.join(_HERE, "..", "..", "tablets_stl", "stl"))

# ── 결과 저장 디렉토리 ────────────────────────────────────────────────
_SIM_RESULT = os.path.normpath(os.path.join(_HERE, "..", "Sim_result"))
CSV_DIR     = os.path.join(_SIM_RESULT, "csv")
PLOT_DIR    = os.path.join(_SIM_RESULT, "plot")

# ── 배치 좌표 (mm, MuJoCo world frame) ──────────────────────────────
PLACE_X_MM = -47.879
PLACE_Z_MM =  50.108
WALL_Y_MM  = 336.199   # impact plate 벽 표면 Y [mm]

# ── Phase 파라미터 ────────────────────────────────────────────────────
PHASE1_STEPS = 500
SIM_DURATION = 30.0
MOTOR_CTRL   = -0.5
MOTOR_DELAY  =  3.0

# ── Moving-window stall 감지 (mirrors Keyborad_control_v2.py) ────────
# Real: rpm_buffer = deque(maxlen=5); stall when sum==0
# Sim : stall_buf  = deque(maxlen=stall_window); stall when all(vel < thr)
STALL_TIME_S  = 2.0
STALL_VEL_THR = 0.05
# Real: set_enable(0) → time.sleep(0.5) → set_enable(1)
# Sim : ctrl=0 for SETTLE_TIME_S steps, then ctrl=motor_dir*MOTOR_CTRL
SETTLE_TIME_S = 0.5    # [s] coast period after stall-triggered direction flip

# ── 실시간 플롯 갱신 주기 ─────────────────────────────────────────────
RT_PLOT_INTERVAL = 20

# ── 알약 초기 자세 (쿼터니언, qw qx qy qz) ──────────────────────────
_s = np.sqrt(2.0) / 2.0
TAB_QUAT_STR = f"{_s:.7f} 0.0000000 {_s:.7f} 0.0000000"

# ── 밀도 기반 접촉 경도 파라미터 ─────────────────────────────────────
DENSITY_REF_SOFT  = 900.0
DENSITY_REF_HARD  = 1800.0
SOLREF_TAU_SOFT   = 0.020
SOLREF_TAU_HARD   = 0.002
DENSITY_DEFAULT   = 1200.0
BICONVEX_VOL_FACTOR = 0.82

# ── slide joint 기본 감쇠 [N·s/m] ────────────────────────────────────
SLIDE_DAMPING_DEFAULT = 10.0   # 낮게 설정 → 자유롭게 밀려남


# ─────────────────────────────────────────────────────────────────────
def _parse_params(fname: str):
    stem = os.path.splitext(os.path.basename(fname))[0]
    m = re.search(r"R([\d.]+)_AR([\d.]+)_CV([\d.]+)", stem)
    if not m:
        return None, None, None
    return float(m.group(1)), float(m.group(2)), float(m.group(3))


def estimate_tablet_volume_mm3(R_mm, AR, CV):
    cd = CV * 2 * R_mm
    th = R_mm * 0.20 + 2 * cd
    a  = R_mm * AR
    b  = R_mm
    c  = th / 2.0
    return (4.0 / 3.0) * math.pi * a * b * c * BICONVEX_VOL_FACTOR


def density_to_solref_tau(density_kg_m3: float) -> float:
    rho   = float(np.clip(density_kg_m3, DENSITY_REF_SOFT, DENSITY_REF_HARD))
    alpha = math.log(SOLREF_TAU_HARD / SOLREF_TAU_SOFT) / \
            math.log(DENSITY_REF_HARD / DENSITY_REF_SOFT)
    tau   = SOLREF_TAU_SOFT * (DENSITY_REF_SOFT / rho) ** alpha
    return float(np.clip(tau, SOLREF_TAU_HARD, SOLREF_TAU_SOFT))


# ─────────────────────────────────────────────────────────────────────
def _build_model(stl_path: str, R_mm: float, half_th: float,
                 density_kg_m3: float = DENSITY_DEFAULT,
                 slide_damping: float = SLIDE_DAMPING_DEFAULT):
    """
    Crusher XML + Tablet STL → MjModel

    알약 body:
      - mocap 없음, freejoint 없음
      - slide joint (Y축, 단방향) 만 허용
      - 뒷벽(L1_Wall1_1)이 이미 XML에 존재 → 알약이 눌리면 벽에 반력 발생
    """
    pos_x = PLACE_X_MM * 1e-3
    pos_z = PLACE_Z_MM * 1e-3
    # 알약 초기 위치: 표면이 벽에 딱 닿도록
    pos_y = (WALL_Y_MM - half_th) * 1e-3

    tau      = density_to_solref_tau(density_kg_m3)
    dimp_max = float(np.interp(density_kg_m3,
                               [DENSITY_REF_SOFT, DENSITY_REF_HARD],
                               [0.950, 0.999]))
    solref_str = f"{tau:.6f} 1"
    solimp_str = f"0.99 {dimp_max:.4f} 0.0001"

    center_y_mm = WALL_Y_MM - half_th
    print(f"  배치 [mm] : X={PLACE_X_MM:.3f}  Y_center={center_y_mm:.3f}  Z={PLACE_Z_MM:.3f}")
    print(f"  배치 [m]  : X={pos_x:.5f}  Y={pos_y:.5f}  Z={pos_z:.5f}")
    print(f"  접촉 경도 : density={density_kg_m3:.0f} kg/m³  "
          f"τ={tau:.5f}s  solimp_dmax={dimp_max:.4f}")
    print(f"  slide 감쇠: {slide_damping:.1f} N·s/m")

    tree = ET.parse(MJCF_PATH)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", MJCF_DIR)

    for kf in root.findall("keyframe"):
        root.remove(kf)

    asset = root.find("asset")
    ET.SubElement(asset, "mesh", {
        "name":  "tablet_mesh",
        "file":  "tablet.stl",
        "scale": ".001 .001 .001",
    })
    ET.SubElement(asset, "material", {
        "name":      "tablet_mat",
        "rgba":      ".85 .80 .72 1",
        "specular":  ".4",
        "shininess": ".3",
    })

    worldbody = root.find("worldbody")

    # ── tablet body: slide joint (Y축만 허용) ─────────────────────────
    tab = ET.SubElement(worldbody, "body", {
        "name": "tablet",
        "pos":  f"{pos_x:.6f} {pos_y:.6f} {pos_z:.6f}",
        "quat": TAB_QUAT_STR,
    })
    ET.SubElement(tab, "joint", {
        "name":    "tablet_slide",
        "type":    "slide",
        "axis":    "0 1 0",          # Y축 슬라이드 (슬라이더가 밀어오는 방향)
        "damping": f"{slide_damping:.4f}",
        "limited":  "false",          # Y 이동 무제한 (뒷벽 geom이 막음)
    })
    ET.SubElement(tab, "geom", {
        "name":     "tablet_geom",
        "type":     "mesh",
        "mesh":     "tablet_mesh",
        "material": "tablet_mat",
        "density":  f"{density_kg_m3:.1f}",
        "condim":   "4",
        "friction": ".5 .02 .01",
        "solref":   solref_str,
        "solimp":   solimp_str,
    })

    xml_str   = ET.tostring(root, encoding="unicode")
    stl_bytes = open(stl_path, "rb").read()
    model     = mujoco.MjModel.from_xml_string(
        xml_str, assets={"tablet.stl": stl_bytes})
    return model, (pos_x, pos_y, pos_z)


# ─────────────────────────────────────────────────────────────────────
def _sum_contact_force(model, data, body_id) -> np.ndarray:
    """body_id에 작용하는 접촉력 합 (world frame XYZ) [N].
    슬라이더 힘 + 벽 반력을 모두 더하므로 준정적 시 net ≈ 0."""
    f_total = np.zeros(3)
    force6  = np.zeros(6)
    for i in range(data.ncon):
        c    = data.contact[i]
        g1_b = model.geom_bodyid[c.geom1]
        g2_b = model.geom_bodyid[c.geom2]
        if body_id not in (g1_b, g2_b):
            continue
        mujoco.mj_contactForce(model, data, i, force6)
        frame   = c.frame.reshape(3, 3)
        f_world = frame.T @ force6[:3]
        if g2_b == body_id:
            f_world = -f_world
        f_total += f_world
    return f_total


def _wall_tablet_force_N(model, data, gid_wall: int, bid_tablet: int) -> float:
    """
    벽(L1_Wall1_1) ↔ 알약 접촉쌍만 분리한 법선 압축력 합 [N].

    - 슬라이더·기타 접촉은 모두 제외
    - force6[0] = 법선력 크기 (항상 ≥ 0, 압축 전용)
    - 이 값 = 경도 시험기(Schleuniger 등)가 측정하는 파괴 하중과 동일한 물리량
    - J = ∫F_wall dt > 0  (단일 접촉쌍 → 슬라이더 반력과 상쇄 없음)
    """
    total_N = 0.0
    force6  = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        is_wall_tab = (
            (c.geom1 == gid_wall and model.geom_bodyid[c.geom2] == bid_tablet) or
            (c.geom2 == gid_wall and model.geom_bodyid[c.geom1] == bid_tablet)
        )
        if not is_wall_tab:
            continue
        mujoco.mj_contactForce(model, data, i, force6)
        total_N += force6[0]   # 법선 압축력 크기
    return total_N


# ─────────────────────────────────────────────────────────────────────
def run(stl_path: str,
        density_kg_m3: float = DENSITY_DEFAULT,
        slide_damping: float = SLIDE_DAMPING_DEFAULT):

    print("=" * 66)
    print("  Crusher + Tablet  2-Phase 시뮬레이션  [slide joint — Y축 자유 이동]")
    print("=" * 66)

    fname = os.path.basename(stl_path)
    R_mm, AR, CV = _parse_params(fname)
    if R_mm is None:
        print(f"[ERROR] 파일명 파싱 실패: {fname}")
        sys.exit(1)

    cd      = CV * 2 * R_mm
    th      = R_mm * 0.20 + 2 * cd
    half_th = th / 2.0
    vol_mm3 = estimate_tablet_volume_mm3(R_mm, AR, CV)
    tau     = density_to_solref_tau(density_kg_m3)

    print(f"  STL  : {fname}")
    print(f"  R={R_mm:.1f}mm  AR={AR:.2f}  CV={CV:.2f}  "
          f"두께≈{th:.2f}mm  half_th={half_th:.2f}mm")
    print(f"  밀도={density_kg_m3:.0f} kg/m³  부피≈{vol_mm3:.1f} mm³  τ={tau:.5f}s")
    print()

    os.makedirs(CSV_DIR,  exist_ok=True)
    os.makedirs(PLOT_DIR, exist_ok=True)
    stem        = os.path.splitext(fname)[0]
    ts          = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_stem = f"slidejoint_{stem}__{ts}"
    print(f"  결과 prefix: {result_stem}\n")

    model, (px, py, pz) = _build_model(
        stl_path, R_mm, half_th, density_kg_m3, slide_damping)
    data = mujoco.MjData(model)

    # ── ID 조회 ──────────────────────────────────────────────────────
    crank_jid   = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT,    "L3_Bevel_GearBox_1_L4_Shaft_1")
    slide_jid   = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT,    "tablet_slide")
    act_crank   = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Motor1_crank")
    b_tablet    = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY,     "tablet")
    b_slider    = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY,     "L8_Link3_Shaft_1")
    eq_lock_id  = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_EQUALITY, "lock_crank")
    gid_wall    = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM,     "L1_Wall1_1")   # 충돌판 벽 geom

    crank_qadr  = model.jnt_qposadr[crank_jid]
    crank_vadr  = model.jnt_dofadr[crank_jid]
    slide_qadr  = model.jnt_qposadr[slide_jid]
    slide_vadr  = model.jnt_dofadr[slide_jid]

    print(f"\n  nq={model.nq}  |  crank qpos[{crank_qadr}]  "
          f"slide qpos[{slide_qadr}]  lock_crank eq={eq_lock_id}")

    # ── ❶ 초기 상태 ──────────────────────────────────────────────────
    data.qpos[crank_qadr] = -np.pi / 2
    data.qpos[slide_qadr] = 0.0          # 슬라이드 변위 0 (초기 위치)
    data.qvel[:]          = 0.0
    mujoco.mj_forward(model, data)

    # ── ❷ Phase 1: 안정화 (뷰어 없음) ───────────────────────────────
    print(f"\n◆ Phase 1: {PHASE1_STEPS} 스텝 안정화")
    for _ in range(PHASE1_STEPS):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    slide_init_disp = float(data.qpos[slide_qadr])
    print(f"  ✔ Phase 1 완료  crank={np.degrees(data.qpos[crank_qadr]):.1f}°"
          f"  slide_disp={slide_init_disp*1e3:.3f} mm  sim_time={data.time:.3f}s")

    # ── ❸ Phase 2 준비 ───────────────────────────────────────────────
    _dt          = float(model.opt.timestep)
    stall_window  = max(1, int(round(STALL_TIME_S  / _dt)))
    settle_steps  = max(1, int(round(SETTLE_TIME_S / _dt)))

    print(f"\n◆ Phase 2: 뷰어 오픈")
    print(f"  {MOTOR_DELAY:.1f}s 후 lock_crank 해제 → {MOTOR_CTRL} N·m CCW")
    print(f"  stall window: {stall_window} 스텝 ({STALL_TIME_S:.1f}s)")
    print(f"  settle steps: {settle_steps} ({SETTLE_TIME_S:.1f}s)  [real: time.sleep(0.5)]")
    print(f"  측정 시간 = {SIM_DURATION} s\n")

    data.ctrl[act_crank] = 0.0
    phase2_start_t = data.time

    motor_on         = False
    motor_on_t       = None
    motor_dir        = 0
    stall_buf        = deque(maxlen=stall_window)
    settle_countdown = 0   # >0 while coasting after direction flip
    rev_events       = []

    t_log        = []
    f_log        = []          # 알약 net 접촉력 (준정적 ≈ 0)
    f_wall_log   = []          # 벽-알약 법선 압축력 (경도 시험기 측정값에 해당)
    slide_disp   = []          # 알약 Y 변위 [m]
    slide_vel_log= []          # 알약 Y 속도 [m/s]
    vel_log      = []          # 크랭크 각속도
    dir_log      = []
    ncon_log     = []
    slider_y_log = []
    tablet_y_log = []
    gap_log      = []
    first_contact_t = None

    print(f"  {'Time':>6s} | {'SliderY':>8s}mm | {'TabletY':>8s}mm | "
          f"{'Gap':>6s}mm | {'F_Y':>8s}N | {'disp':>7s}mm | {'ncon':>4s}")
    print("  " + "-" * 80)

    # ── 실시간 반력 플롯 ──────────────────────────────────────────────
    plt.ion()
    fig_rt, axes_rt = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    fig_rt.suptitle(
        f"실시간 반력 / 알약 변위  [slide joint]  ρ={density_kg_m3:.0f} kg/m³",
        fontsize=10)
    line_fy,   = axes_rt[0].plot([], [], color="tab:red",    lw=1.5, label="F_wall [N]  (벽-알약 압축력)")
    line_disp, = axes_rt[1].plot([], [], color="tab:orange", lw=1.5, label="Tablet 변위 [mm]")
    axes_rt[0].axhline(0, color="k", lw=0.5, ls="--")
    axes_rt[1].axhline(0, color="k", lw=0.5, ls="--")
    axes_rt[0].set_ylabel("F_wall [N]"); axes_rt[0].legend(fontsize=9); axes_rt[0].grid(True, alpha=0.3)
    axes_rt[1].set_ylabel("변위 [mm]");  axes_rt[1].legend(fontsize=9); axes_rt[1].grid(True, alpha=0.3)
    axes_rt[1].set_xlabel("Time [s]")
    fig_rt.tight_layout()
    fig_rt.canvas.draw()
    fig_rt.canvas.flush_events()
    _rt_vlines = []

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT]      = False
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = False
        viewer.opt.frame                                      = mujoco.mjtFrame.mjFRAME_NONE
        viewer.opt.geomgroup[3]                               = False
        for sg in range(5):
            viewer.opt.sitegroup[sg] = False

        while viewer.is_running() and data.time < SIM_DURATION:

            # 모터 ON / lock 해제
            if not motor_on and (data.time - phase2_start_t) >= MOTOR_DELAY:
                data.eq_active[eq_lock_id] = 0
                motor_dir = 1
                data.ctrl[act_crank] = motor_dir * MOTOR_CTRL
                motor_on   = True
                motor_on_t = data.time
                stall_buf.clear()
                print(f"  *** lock 해제 + 모터 ON: t={data.time:.3f}s  "
                      f"ctrl={motor_dir * MOTOR_CTRL:.2f} N·m ***")

            # Moving-window stall 감지 + settle (mirrors Keyborad_control_v2.py)
            if motor_on:
                if settle_countdown > 0:
                    # enable(0) 상태: ctrl=0 유지 (real: set_enable(0) + time.sleep(0.5))
                    data.ctrl[act_crank] = 0.0
                    settle_countdown -= 1
                    if settle_countdown == 0:
                        # enable(1) 상태로 전환: 새 방향으로 구동 시작
                        data.ctrl[act_crank] = motor_dir * MOTOR_CTRL
                        stall_buf.clear()
                        dir_str = "CCW" if motor_dir > 0 else "CW"
                        print(f"  *** Settle 완료 → {dir_str} ON  t={data.time:.3f}s ***")
                else:
                    crank_vel = abs(data.qvel[crank_vadr])
                    stall_buf.append(crank_vel < STALL_VEL_THR)
                    if len(stall_buf) == stall_window and all(stall_buf):
                        # Stall 감지: enable(0) → 방향 반전 → settle → enable(1)
                        motor_dir = -motor_dir
                        data.ctrl[act_crank] = 0.0   # enable OFF
                        settle_countdown = settle_steps
                        stall_buf.clear()
                        dir_str = "CCW" if motor_dir > 0 else "CW"
                        rev_events.append((data.time, dir_str))
                        print(f"  *** Stall → {dir_str} (settle {SETTLE_TIME_S:.1f}s)"
                              f"  t={data.time:.3f}s ***")

            mujoco.mj_step(model, data)

            sy      = float(data.xpos[b_slider, 1])
            ty_now  = float(data.xpos[b_tablet, 1])
            gap_mm  = (ty_now - sy) * 1e3
            omega   = float(data.qvel[crank_vadr])
            s_disp  = float(data.qpos[slide_qadr])   # 슬라이드 변위 [m]
            s_vel   = float(data.qvel[slide_vadr])
            fc_now     = _sum_contact_force(model, data, b_tablet)
            fw_now     = _wall_tablet_force_N(model, data, gid_wall, b_tablet)

            t_log.append(data.time)
            f_log.append(fc_now.copy())
            f_wall_log.append(fw_now)
            slide_disp.append(s_disp)
            slide_vel_log.append(s_vel)
            vel_log.append(omega)
            dir_log.append(motor_dir)
            ncon_log.append(data.ncon)
            slider_y_log.append(sy)
            tablet_y_log.append(ty_now)
            gap_log.append(gap_mm)

            if first_contact_t is None and fw_now > 0.1:
                first_contact_t = data.time
                print(f"  *** 첫 벽-알약 접촉: t={data.time:.3f}s  "
                      f"F_wall={fw_now:.2f}N  disp={s_disp*1e3:.3f}mm ***")

            if len(t_log) % 500 == 0:
                dir_lbl = {1: "CCW", -1: "CW", 0: "---"}.get(motor_dir, "?")
                print(f"  {data.time:6.2f}s | {sy*1e3:8.2f}   | "
                      f"{ty_now*1e3:8.2f}   | {gap_mm:6.2f}   | "
                      f"{fw_now:8.2f} | {s_disp*1e3:7.3f}   | {data.ncon:4d}"
                      f"  [{dir_lbl}]")

            # 실시간 플롯 갱신 — F_wall (벽-알약 압축력) 표시
            if len(t_log) % RT_PLOT_INTERVAL == 0 and len(t_log) > 1:
                disp_data = [d * 1e3 for d in slide_disp]
                line_fy.set_data(t_log, f_wall_log)   # F_wall 로 교체
                line_disp.set_data(t_log, disp_data)
                for ax_ in axes_rt:
                    ax_.relim(); ax_.autoscale_view()
                for vl in _rt_vlines:
                    try: vl.remove()
                    except Exception: pass
                _rt_vlines.clear()
                if motor_on_t is not None:
                    for ax_ in axes_rt:
                        _rt_vlines.append(ax_.axvline(
                            motor_on_t, color="tab:orange", ls="--", lw=1.0))
                for ev_t, _ in rev_events:
                    for ax_ in axes_rt:
                        _rt_vlines.append(ax_.axvline(
                            ev_t, color="tab:red", ls=":", lw=1.0))
                fig_rt.canvas.draw_idle()
                fig_rt.canvas.flush_events()

            viewer.sync()

    plt.ioff()

    # ── 결과 집계 ─────────────────────────────────────────────────────
    if not t_log:
        print("[경고] 데이터 없음.")
        return

    t    = np.array(t_log)
    fc      = np.array(f_log)
    f_wall  = np.array(f_wall_log)   # 벽-알약 압축력 [N], 항상 ≥ 0
    disp    = np.array(slide_disp) * 1e3   # mm
    svel    = np.array(slide_vel_log)
    vel     = np.array(vel_log)
    sy      = np.array(slider_y_log) * 1e3
    ty      = np.array(tablet_y_log) * 1e3
    gap     = np.array(gap_log)
    fc_mag  = np.linalg.norm(fc, axis=1)

    # ── 핵심 지표 ──────────────────────────────────────────────────────
    # F_wall: 벽-알약 접촉 법선력 (단일 contact pair → 상쇄 없음 → J > 0)
    J_wall      = float(_np_trapz(f_wall, t))       # 실질 충격량 [N·s]
    F_wall_max  = float(f_wall.max())               # 최대 압축력 (= 경도 지표)
    # fc: 알약 net 접촉력 (슬라이더+벽 합산 → 준정적 ≈ 0, 참고용)
    J_Y         = float(_np_trapz(fc[:, 1], t))
    F_Y_max     = float(fc[:, 1].max())
    F_Y_min     = float(fc[:, 1].min())
    disp_max    = float(disp.max())

    print(f"\n  {'='*66}")
    print(f"  수집      : {len(t)} steps  ({t[-1]:.2f} s)")
    print(f"  Slider Y  : {sy.min():.1f} ~ {sy.max():.1f} mm")
    print(f"  Tablet Y  : {ty.min():.1f} ~ {ty.max():.1f} mm")
    print(f"  Slide 변위: {disp.min():.3f} ~ {disp.max():.3f} mm  (+ = 벽 방향)")
    print(f"  Min gap   : {gap.min():.2f} mm  (<0 = 관통)")
    print(f"  ── 벽-알약 접촉 (단일 contact pair) ──")
    print(f"  F_wall max: {F_wall_max:.3f} N  ← 경도 지표 (파괴 하중 대응)")
    print(f"  J_wall    : {J_wall:.5f} N·s  ← 실질 충격량 (> 0 보장)")
    print(f"  ── net 접촉력 (참고, 준정적 ≈ 0) ──")
    print(f"  F_Y range : {F_Y_min:.3f} ~ {F_Y_max:.3f} N")
    print(f"  J_Y (net) : {J_Y:.5f} N·s")
    print(f"  방향 전환 : {len(rev_events)} 회")
    if first_contact_t:
        print(f"  첫 접촉   : t = {first_contact_t:.3f} s")
    else:
        print("  [!] 접촉 없음")
    print(f"  {'='*66}")

    # ── 플롯 ─────────────────────────────────────────────────────────
    title_base = (f"[slide joint]  Motor={MOTOR_CTRL} N·m  |  "
                  f"R={R_mm:.1f} AR={AR:.2f} CV={CV:.2f}  |  "
                  f"ρ={density_kg_m3:.0f} kg/m³  damp={slide_damping:.0f}")

    def _vlines(ax_):
        if motor_on_t is not None:
            ax_.axvline(motor_on_t, color="tab:orange", ls="--", lw=1.2,
                        label=f"모터 ON (t={motor_on_t:.2f}s)")
        for ev_t, ev_dir in rev_events:
            ax_.axvline(ev_t, color="tab:red", ls=":", lw=1.0,
                        label=f"→{ev_dir}")

    # 그림 1: 위치 + 변위
    fig1, axes1 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig1.suptitle(f"Position & Displacement — {title_base}", fontsize=9, fontweight="bold")
    axes1[0].plot(t, sy, color="tab:orange", lw=1.5, label="Slider Y")
    axes1[0].plot(t, ty, color="tab:blue",   lw=1.5, label="Tablet Y")
    axes1[0].axhline(WALL_Y_MM, color="tab:red", ls=":", lw=1.2, label=f"Wall={WALL_Y_MM:.1f}mm")
    _vlines(axes1[0]); axes1[0].set_ylabel("World Y [mm]"); axes1[0].legend(fontsize=8); axes1[0].grid(True, alpha=0.3)
    axes1[1].plot(t, gap, color="tab:purple", lw=1.5)
    axes1[1].axhline(0, color="tab:red", ls="--", lw=0.8, label="Contact")
    _vlines(axes1[1]); axes1[1].set_ylabel("Gap [mm]"); axes1[1].legend(fontsize=8); axes1[1].grid(True, alpha=0.3)
    axes1[2].plot(t, disp, color="tab:green", lw=1.5, label="Tablet 변위 [mm]")
    axes1[2].axhline(0, color="k", lw=0.5)
    _vlines(axes1[2]); axes1[2].set_ylabel("변위 [mm]"); axes1[2].set_xlabel("Time [s]")
    axes1[2].legend(fontsize=8); axes1[2].grid(True, alpha=0.3)
    fig1.tight_layout()

    # 그림 2: 벽-알약 압축력 + 누적 충격량
    fig2, axes2 = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    fig2.suptitle(f"Wall-Tablet Contact Force — {title_base}", fontsize=9, fontweight="bold")

    # 2-0: 벽-알약 압축력 (단일 contact pair, 항상 ≥ 0)
    axes2[0].plot(t, f_wall, color="tab:red", lw=1.5,
                  label=f"F_wall [N]  (max={F_wall_max:.2f} N)")
    axes2[0].fill_between(t, 0, f_wall, alpha=0.15, color="tab:red")
    _vlines(axes2[0]); axes2[0].set_ylabel("F_wall [N]")
    axes2[0].set_title(f"벽-알약 압축력  (경도 시험기 측정값)  max = {F_wall_max:.2f} N",
                       fontsize=9)
    axes2[0].legend(fontsize=8); axes2[0].grid(True, alpha=0.3)

    # 2-1: net F_Y (참고용, ≈ 0)
    axes2[1].plot(t, fc[:, 1], color="tab:blue", lw=1.0, alpha=0.7, label="net F_Y [N]  (참고)")
    axes2[1].fill_between(t, 0, fc[:, 1], where=(fc[:, 1] > 0), alpha=0.10, color="tab:blue", label="압축")
    axes2[1].fill_between(t, 0, fc[:, 1], where=(fc[:, 1] < 0), alpha=0.10, color="tab:red",  label="인장")
    _vlines(axes2[1]); axes2[1].set_ylabel("net F_Y [N]")
    axes2[1].set_title(f"알약 합력 (슬라이더+벽 합산, 준정적 ≈ 0)  net_J={J_Y:.4f} N·s",
                       fontsize=9)
    axes2[1].legend(fontsize=8); axes2[1].grid(True, alpha=0.3)

    # 2-2: 누적 충격량
    dt_sim = float(model.opt.timestep)
    J_wall_cumul = np.cumsum(f_wall) * dt_sim
    axes2[2].plot(t, J_wall_cumul, color="tab:red",   lw=1.5,
                  label=f"J_wall = {J_wall:.4f} N·s")
    axes2[2].plot(t, np.cumsum(fc[:, 1]) * dt_sim, color="tab:blue", lw=1.0,
                  ls="--", alpha=0.6, label=f"J_net  = {J_Y:.4f} N·s  (참고)")
    _vlines(axes2[2]); axes2[2].set_ylabel("Impulse [N·s]"); axes2[2].set_xlabel("Time [s]")
    axes2[1].legend(fontsize=8); axes2[1].grid(True, alpha=0.3)
    fig2.tight_layout()

    # 그림 3: XYZ 성분
    fig3, axes3 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig3.suptitle("Contact Force Components — " + title_base, fontsize=9, fontweight="bold")
    for i, (lbl, col) in enumerate(
            [("X (lateral)", "tab:red"),
             ("Y (normal)",  "tab:blue"),
             ("Z (vertical)","tab:green")]):
        axes3[i].plot(t, fc[:, i], color=col, lw=1.2, label=f"F_{lbl}")
        axes3[i].axhline(0, color="k", lw=0.5)
        _vlines(axes3[i]); axes3[i].set_ylabel("F [N]")
        axes3[i].legend(fontsize=8); axes3[i].grid(True, alpha=0.3)
    axes3[2].set_xlabel("Time [s]")
    fig3.tight_layout()

    # 그림 4: 알약 변위 + 속도
    fig4, axes4 = plt.subplots(2, 1, figsize=(11, 5), sharex=True)
    fig4.suptitle(f"Tablet Slide Motion — {title_base}", fontsize=9, fontweight="bold")
    axes4[0].plot(t, disp, color="tab:orange", lw=1.5, label="변위 [mm]")
    _vlines(axes4[0]); axes4[0].set_ylabel("변위 [mm]"); axes4[0].legend(fontsize=8); axes4[0].grid(True, alpha=0.3)
    axes4[1].plot(t, svel * 1e3, color="tab:purple", lw=1.2, label="속도 [mm/s]")
    axes4[1].axhline(0, color="k", lw=0.5)
    _vlines(axes4[1]); axes4[1].set_ylabel("속도 [mm/s]"); axes4[1].set_xlabel("Time [s]")
    axes4[1].legend(fontsize=8); axes4[1].grid(True, alpha=0.3)
    fig4.tight_layout()

    # ── CSV 저장 ──────────────────────────────────────────────────────
    csv_path = os.path.join(CSV_DIR, f"{result_stem}.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["# Crusher Tablet Simulation — slide joint"])
        w.writerow(["# Generated", datetime.now().isoformat(timespec="seconds")])
        w.writerow(["# STL", fname, "R_mm", R_mm, "AR", AR, "CV", CV])
        w.writerow(["# density_kg_m3", density_kg_m3, "slide_damping_Nsm", slide_damping])
        w.writerow(["# solref_tau_s", f"{tau:.6f}"])
        w.writerow(["# MOTOR_CTRL_Nm", MOTOR_CTRL, "MOTOR_DELAY_s", MOTOR_DELAY])
        w.writerow(["# F_Y_max_N", f"{F_Y_max:.5f}", "F_Y_min_N", f"{F_Y_min:.5f}",
                    "Impulse_Ns", f"{J_Y:.6f}", "disp_max_mm", f"{disp_max:.4f}"])
        w.writerow([])
        w.writerow(["Time_s", "F_X_N", "F_Y_N", "F_Z_N", "F_mag_N",
                    "Slider_Y_mm", "Tablet_Y_mm", "Gap_mm",
                    "Slide_disp_mm", "Slide_vel_mms",
                    "Crank_vel_rads", "Motor_dir", "ncon"])
        _DIR_STR = {1: "CCW", -1: "CW", 0: "off"}
        for i in range(len(t_log)):
            w.writerow([
                f"{t_log[i]:.5f}",
                f"{f_log[i][0]:.6f}", f"{f_log[i][1]:.6f}", f"{f_log[i][2]:.6f}",
                f"{fc_mag[i]:.6f}",
                f"{slider_y_log[i]*1e3:.4f}", f"{tablet_y_log[i]*1e3:.4f}",
                f"{gap_log[i]:.4f}",
                f"{slide_disp[i]*1e3:.5f}", f"{slide_vel_log[i]*1e3:.5f}",
                f"{vel_log[i]:.6f}",
                _DIR_STR.get(dir_log[i], "?"), ncon_log[i],
            ])
    print(f"\n  ✔ CSV: {csv_path}")

    plot_specs = [
        (fig_rt, "realtime"),
        (fig1,   "position"),
        (fig2,   "force_fy"),
        (fig3,   "force_components"),
        (fig4,   "tablet_motion"),
    ]
    for fig, tag in plot_specs:
        p = os.path.join(PLOT_DIR, f"{result_stem}__{tag}.png")
        fig.savefig(p, dpi=150, bbox_inches="tight")
        print(f"     {os.path.basename(p)}")
    plt.show()


# ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Crusher + Tablet — slide joint (Y축) 방식",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "예시:\n"
            "  python crusher_tablet_slidejoint.py tablet_R6.0_AR1.50_CV0.20.stl\n"
            "  python crusher_tablet_slidejoint.py tablet.stl --density 1400\n"
            "  python crusher_tablet_slidejoint.py tablet.stl --damping 50\n"
        ),
    )
    parser.add_argument("stl", nargs="?", default=None, help="Tablet STL 경로")
    parser.add_argument("--density", type=float, default=None, metavar="KG_M3",
                        help=f"알약 밀도 [kg/m³] (기본: {DENSITY_DEFAULT:.0f})")
    parser.add_argument("--damping", type=float, default=SLIDE_DAMPING_DEFAULT,
                        metavar="NS_M",
                        help=f"slide joint 감쇠 [N·s/m] (기본: {SLIDE_DAMPING_DEFAULT:.0f})")
    args = parser.parse_args()
    stl_path = args.stl

    if stl_path is None:
        try:
            import tkinter as tk
            from tkinter import filedialog
            root_tk = tk.Tk(); root_tk.withdraw()
            stl_path = filedialog.askopenfilename(
                title="Tablet STL 선택",
                initialdir=STL_DIR if os.path.isdir(STL_DIR) else "~",
                filetypes=[("STL Files", "*.stl")],
            )
            root_tk.destroy()
        except Exception:
            pass

    if not stl_path:
        print("사용법: python crusher_tablet_slidejoint.py <path>.stl [옵션]")
        sys.exit(0)

    stl_path = os.path.abspath(stl_path)
    if not os.path.exists(stl_path):
        print(f"[오류] 파일 없음: {stl_path}")
        sys.exit(1)

    density_kg_m3 = DENSITY_DEFAULT if args.density is None else float(args.density)
    run(stl_path, density_kg_m3=density_kg_m3, slide_damping=args.damping)

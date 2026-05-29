"""
batch_tablet_sim.py  [v1 — headless 1000-tablet batch]
1000개 정제 형상에 대한 일괄 Crusher 시뮬레이션 (뷰어 없음)

▶ 방법 A: 순차(sequential) — 기본 모드
    STL 파일을 하나씩 로드 → 헤드리스 물리 스텝만 실행 → CSV 저장
    뷰어/렌더링 없음 → 인터랙티브 대비 10~50배 빠름

▶ 방법 B: 병렬(parallel) — --parallel 플래그
    multiprocessing.Pool 으로 N_CPU 개 프로세스 동시 실행
    각 프로세스: 독립된 MjModel + MjData → 완전 안전

▶ mocap body 병렬 안전성 (검증 완료)
    data.contact, mocap_pos/quat, mj_contactForce 모두
    per-MjData 인스턴스 → 프로세스 간 공유 전역 상태 없음

▶ 출력 구조
    Sim_result/
    ├── csv/
    │   ├── {batch_ts}/
    │   │   ├── {stem}.csv          ← 개별 반력 프로파일 (스텝별)
    │   │   └── ...
    │   └── summary__{batch_ts}.csv ← 요약 (행 = 1개 정제)
    └── plot/
        ├── {batch_ts}/
        │   ├── {stem}__force.png   ← 개별 플롯 (--save-plots 시)
        │   └── ...
        └── summary__{batch_ts}.png ← 요약 산포도

실행:
    conda activate isaac_sim
    # 순차 실행 (1000개, 각 10s)
    python batch_tablet_sim.py

    # 병렬 실행 (8 워커)
    python batch_tablet_sim.py --parallel --workers 8

    # 빠른 테스트 (10개만, plot 저장)
    python batch_tablet_sim.py --limit 10 --save-plots

    # 전체 옵션
    python batch_tablet_sim.py --duration 10 --parallel --save-plots --stl-dir <path>
"""

import os
import sys
import re
import csv
import glob
import time
import argparse
import traceback
import multiprocessing
import xml.etree.ElementTree as ET
from collections import deque
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use("Agg")        # 헤드리스 백엔드: 화면 출력 없이 파일 저장만
import matplotlib.pyplot as plt
import mujoco

# ── 경로 ─────────────────────────────────────────────────────────────
_HERE     = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.normpath(
    os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml"))
MJCF_DIR  = os.path.dirname(MJCF_PATH)
STL_DIR   = os.path.normpath(
    os.path.join(_HERE, "..", "..", "tablets_stl", "stl"))
_SIM_RESULT = os.path.normpath(os.path.join(_HERE, "..", "Sim_result"))
CSV_DIR     = os.path.join(_SIM_RESULT, "csv")
PLOT_DIR    = os.path.join(_SIM_RESULT, "plot")

# ── 시뮬레이션 고정 파라미터 ─────────────────────────────────────────
PLACE_X_MM    = -47.879
PLACE_Z_MM    =  50.108
WALL_Y_MM     = 336.199   # impact plate 벽 표면 Y [mm]
#   알약 중심 Y = WALL_Y_MM - R_mm  (단반경만큼 벽 앞에 배치)
PHASE1_STEPS  = 500
MOTOR_CTRL    = -0.5      # [N·m]  음수 = CCW
MOTOR_DELAY   =  3.0      # [s]
STALL_WINDOW  = 30
STALL_VEL_THR = 0.05      # [rad/s]
_s = np.sqrt(2.0) / 2.0
TAB_QUAT = np.array([_s, 0.0, _s, 0.0])    # X90 + Y90 + Z90 합성


# ─────────────────────────────────────────────────────────────────────
# 헬퍼 함수 (모두 module-level → pickle 가능)
# ─────────────────────────────────────────────────────────────────────

def _parse_params(fname: str):
    stem = os.path.splitext(os.path.basename(fname))[0]
    m = re.search(r"R([\d.]+)_AR([\d.]+)_CV([\d.]+)", stem)
    if not m:
        return None, None, None
    return float(m.group(1)), float(m.group(2)), float(m.group(3))


def _build_model(stl_path: str, R_mm: float):
    pos_x = PLACE_X_MM * 1e-3
    pos_z = PLACE_Z_MM * 1e-3
    # 알약 중심 Y = 벽면 - 단반경(R_mm)
    # TAB_QUAT 기준 World-Y 방향 반경 = R_mm → 중심 오프셋 적용
    pos_y = (WALL_Y_MM - R_mm) * 1e-3

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
        "name": "tablet_mesh", "file": "tablet.stl",
        "scale": ".001 .001 .001",
    })
    ET.SubElement(asset, "material", {
        "name": "tablet_mat", "rgba": ".85 .80 .72 1",
        "specular": ".4", "shininess": ".3",
    })

    worldbody = root.find("worldbody")
    tab = ET.SubElement(worldbody, "body", {
        "name": "tablet", "mocap": "true",
        "pos": f"{pos_x:.6f} {pos_y:.6f} {pos_z:.6f}",
        "quat": " ".join(f"{v:.7f}" for v in TAB_QUAT),
    })
    ET.SubElement(tab, "geom", {
        "name": "tablet_geom", "type": "mesh", "mesh": "tablet_mesh",
        "material": "tablet_mat", "density": "1200",
        "condim": "4", "friction": ".5 .02 .01",
    })

    xml_str   = ET.tostring(root, encoding="unicode")
    stl_bytes = open(stl_path, "rb").read()
    return mujoco.MjModel.from_xml_string(
        xml_str, assets={"tablet.stl": stl_bytes}), (pos_x, pos_y, pos_z)


def _sum_contact_force(model, data, body_id) -> np.ndarray:
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


# ─────────────────────────────────────────────────────────────────────
# 핵심 worker: module-level이어야 multiprocessing.Pool에서 pickle 가능
# ─────────────────────────────────────────────────────────────────────

def run_headless(task: tuple) -> dict:
    """
    헤드리스 단일 태블릿 시뮬레이션.
    task = (stl_path, sim_duration, save_plots, batch_csv_dir, batch_plot_dir)
    반환: 결과 dict (에러 시 'error' 키 포함)
    """
    stl_path, sim_duration, save_plots, batch_csv_dir, batch_plot_dir = task
    t_wall0 = time.time()

    fname = os.path.basename(stl_path)
    stem  = os.path.splitext(fname)[0]
    R_mm, AR, CV = _parse_params(fname)

    if R_mm is None:
        return _err(stem, stl_path, R_mm, AR, CV,
                    f"파일명 파싱 실패: {fname}", t_wall0)

    try:
        model, (px, py, pz) = _build_model(stl_path, R_mm)
    except Exception as e:
        return _err(stem, stl_path, R_mm, AR, CV,
                    f"모델 빌드 실패: {e}", t_wall0)

    try:
        result = _simulate(model, stl_path, fname, stem,
                           R_mm, AR, CV, px, py, pz,
                           sim_duration, save_plots,
                           batch_csv_dir, batch_plot_dir, t_wall0)
    except Exception as e:
        return _err(stem, stl_path, R_mm, AR, CV,
                    f"시뮬레이션 오류: {e}\n{traceback.format_exc()}", t_wall0)

    return result


def _err(stem, stl_path, R_mm, AR, CV, msg, t_wall0):
    return {
        "stem": stem, "stl": stl_path,
        "R_mm": R_mm, "AR": AR, "CV": CV,
        "error": msg,
        "wall_time_s": time.time() - t_wall0,
        "F_Y_max_N": None, "F_Y_min_N": None, "F_mag_max_N": None,
        "Impulse_J_Y_Ns": None, "first_contact_s": None,
        "direction_changes": None, "n_steps": 0, "sim_time_s": 0,
        "csv_path": None,
    }


def _simulate(model, stl_path, fname, stem,
              R_mm, AR, CV, px, py, pz,
              sim_duration, save_plots,
              batch_csv_dir, batch_plot_dir, t_wall0):

    data = mujoco.MjData(model)

    # ── ID 조회 ──────────────────────────────────────────────────────
    crank_jid  = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT,    "L3_Bevel_GearBox_1_L4_Shaft_1")
    act_crank  = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Motor1_crank")
    b_tablet   = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY,     "tablet")
    b_slider   = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY,     "L8_Link3_Shaft_1")
    eq_lock_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_EQUALITY, "lock_crank")
    crank_qadr = model.jnt_qposadr[crank_jid]
    crank_vadr = model.jnt_dofadr[crank_jid]
    mocap_id   = model.body_mocapid[b_tablet]

    # ── 초기화 ───────────────────────────────────────────────────────
    data.qpos[crank_qadr]     = -np.pi / 2
    data.qvel[:]              = 0.0
    data.mocap_pos[mocap_id]  = [px, py, pz]
    data.mocap_quat[mocap_id] = TAB_QUAT
    mujoco.mj_forward(model, data)

    # ── Phase 1: 안정화 (뷰어 없음) ──────────────────────────────────
    for _ in range(PHASE1_STEPS):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    # ── Phase 2: 헤드리스 시뮬레이션 ─────────────────────────────────
    data.ctrl[act_crank] = 0.0
    phase2_start_t = data.time

    motor_on   = False
    motor_on_t = None
    motor_dir  = 0
    stall_buf  = deque(maxlen=STALL_WINDOW)
    rev_events = []

    t_log    = []
    f_log    = []
    vel_log  = []
    dir_log  = []
    ncon_log = []
    slider_y = []
    tablet_y = []
    gap_log  = []
    first_contact_t = None

    while data.time < sim_duration:
        # 모터 ON
        if not motor_on and (data.time - phase2_start_t) >= MOTOR_DELAY:
            data.eq_active[eq_lock_id] = 0
            motor_dir = 1
            data.ctrl[act_crank] = motor_dir * MOTOR_CTRL
            motor_on   = True
            motor_on_t = data.time
            stall_buf.clear()

        # Moving-window stall 감지 → 방향 전환
        if motor_on:
            crank_vel = abs(data.qvel[crank_vadr])
            stall_buf.append(crank_vel < STALL_VEL_THR)
            if len(stall_buf) == STALL_WINDOW and all(stall_buf):
                motor_dir = -motor_dir
                data.ctrl[act_crank] = motor_dir * MOTOR_CTRL
                stall_buf.clear()
                rev_events.append((data.time,
                                   "CCW" if motor_dir > 0 else "CW"))

        mujoco.mj_step(model, data)

        sy     = float(data.xpos[b_slider, 1])
        ty_now = float(data.xpos[b_tablet, 1])
        gap_mm = (ty_now - sy) * 1e3
        omega  = float(data.qvel[crank_vadr])
        fc_now = _sum_contact_force(model, data, b_tablet)

        t_log.append(data.time)
        f_log.append(fc_now.copy())
        vel_log.append(omega)
        dir_log.append(motor_dir)
        ncon_log.append(data.ncon)
        slider_y.append(sy)
        tablet_y.append(ty_now)
        gap_log.append(gap_mm)

        if first_contact_t is None and data.ncon > 0:
            for ci in range(data.ncon):
                c = data.contact[ci]
                if b_tablet in (model.geom_bodyid[c.geom1],
                                model.geom_bodyid[c.geom2]):
                    first_contact_t = data.time
                    break

    # ── 결과 집계 ─────────────────────────────────────────────────────
    if not t_log:
        return _err(stem, stl_path, R_mm, AR, CV, "데이터 없음", t_wall0)

    t      = np.array(t_log)
    fc     = np.array(f_log)
    vel    = np.array(vel_log)
    sl_mm  = np.array(slider_y) * 1e3
    gap    = np.array(gap_log)
    fc_mag = np.linalg.norm(fc, axis=1)

    F_Y_max   = float(fc[:, 1].max())
    F_Y_min   = float(fc[:, 1].min())
    F_mag_max = float(fc_mag.max())
    J_Y       = float(np.trapz(fc[:, 1], t))

    # ── 개별 CSV 저장 ─────────────────────────────────────────────────
    _DIR_STR = {1: "CCW", -1: "CW", 0: "off"}
    os.makedirs(batch_csv_dir, exist_ok=True)
    csv_path = os.path.join(batch_csv_dir, f"{stem}.csv")

    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        # 메타데이터
        w.writerow(["# Crusher Tablet Simulation — Force Profile (headless)"])
        w.writerow(["# Generated", datetime.now().isoformat(timespec="seconds")])
        w.writerow(["# STL", fname,
                    "R_mm", R_mm, "AR", AR, "CV", CV])
        w.writerow(["# timestep_s", float(model.opt.timestep),
                    "sim_duration_s", sim_duration,
                    "phase1_steps", PHASE1_STEPS])
        w.writerow(["# MOTOR_CTRL_Nm", MOTOR_CTRL,
                    "MOTOR_DELAY_s", MOTOR_DELAY])
        w.writerow(["# STALL_WINDOW", STALL_WINDOW,
                    "STALL_VEL_THR_rad_s", STALL_VEL_THR])
        w.writerow(["# F_Y_max_N", f"{F_Y_max:.5f}",
                    "F_Y_min_N", f"{F_Y_min:.5f}",
                    "F_mag_max_N", f"{F_mag_max:.5f}",
                    "Impulse_J_Y_Ns", f"{J_Y:.6f}"])
        w.writerow(["# n_rev", len(rev_events)])
        for i, (ev_t, ev_dir) in enumerate(rev_events, 1):
            w.writerow([f"#   rev[{i}]", f"t={ev_t:.4f}s", f"dir={ev_dir}"])
        if first_contact_t:
            w.writerow(["# first_contact_s", f"{first_contact_t:.5f}"])
        w.writerow([])
        # 데이터 헤더
        w.writerow(["Time_s",
                    "F_X_N", "F_Y_N", "F_Z_N", "F_mag_N",
                    "Slider_Y_mm", "Tablet_Y_mm", "Gap_mm",
                    "Crank_vel_rad_s", "Motor_dir", "ncon"])
        for i in range(len(t)):
            w.writerow([
                f"{t[i]:.5f}",
                f"{f_log[i][0]:.6f}",
                f"{f_log[i][1]:.6f}",
                f"{f_log[i][2]:.6f}",
                f"{fc_mag[i]:.6f}",
                f"{sl_mm[i]:.4f}",
                f"{tablet_y[i]*1e3:.4f}",
                f"{gap[i]:.4f}",
                f"{vel[i]:.6f}",
                _DIR_STR.get(dir_log[i], "?"),
                ncon_log[i],
            ])

    # ── 개별 플롯 저장 (--save-plots 시) ─────────────────────────────
    if save_plots:
        os.makedirs(batch_plot_dir, exist_ok=True)
        _save_individual_plot(
            stem, t, fc, vel, gap, fc_mag,
            F_Y_max, J_Y, motor_on_t, rev_events,
            R_mm, AR, CV, batch_plot_dir)

    return {
        "stem": stem, "stl": stl_path,
        "R_mm": R_mm, "AR": AR, "CV": CV,
        "F_Y_max_N":      F_Y_max,
        "F_Y_min_N":      F_Y_min,
        "F_mag_max_N":    F_mag_max,
        "Impulse_J_Y_Ns": J_Y,
        "first_contact_s": first_contact_t,
        "direction_changes": len(rev_events),
        "n_steps":      len(t),
        "sim_time_s":   float(t[-1]),
        "wall_time_s":  time.time() - t_wall0,
        "csv_path":     csv_path,
        "error":        None,
    }


def _save_individual_plot(stem, t, fc, vel, gap, fc_mag,
                          F_Y_max, J_Y, motor_on_t, rev_events,
                          R_mm, AR, CV, batch_plot_dir):
    """개별 태블릿 3-패널 플롯 (Agg 백엔드로 파일 저장)."""
    title = f"R={R_mm:.1f}mm  AR={AR:.2f}  CV={CV:.2f}"
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(f"Force Profile — {title}", fontsize=11, fontweight="bold")

    def _vl(ax_):
        if motor_on_t:
            ax_.axvline(motor_on_t, color="tab:orange", ls="--", lw=1.0,
                        label=f"Motor ON ({motor_on_t:.1f}s)")
        for ev_t, ev_dir in rev_events:
            ax_.axvline(ev_t, color="tab:red", ls=":", lw=0.8)

    # F_Y
    axes[0].plot(t, fc[:, 1], color="tab:blue", lw=1.2, label="F_Y [N]")
    axes[0].fill_between(t, 0, fc[:, 1], where=(fc[:, 1] > 0),
                          alpha=0.12, color="tab:blue")
    axes[0].fill_between(t, 0, fc[:, 1], where=(fc[:, 1] < 0),
                          alpha=0.12, color="tab:red")
    _vl(axes[0])
    axes[0].axhline(0, color="k", lw=0.5)
    axes[0].set_ylabel("F_Y [N]")
    axes[0].set_title(f"max={F_Y_max:.3f} N    J_Y={J_Y:.4f} N·s", fontsize=9)
    axes[0].legend(fontsize=8); axes[0].grid(True, alpha=0.3)

    # Gap
    axes[1].plot(t, gap, color="tab:purple", lw=1.2, label="Gap [mm]")
    axes[1].axhline(0, color="tab:red", ls="--", lw=0.8, label="Contact")
    _vl(axes[1])
    axes[1].set_ylabel("Gap [mm]"); axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    # ω
    axes[2].plot(t, vel, color="tab:green", lw=1.2, label="ω [rad/s]")
    axes[2].axhline( STALL_VEL_THR, color="gray", ls="--", lw=0.7,
                     label=f"stall ±{STALL_VEL_THR} rad/s")
    axes[2].axhline(-STALL_VEL_THR, color="gray", ls="--", lw=0.7)
    axes[2].axhline(0, color="k", lw=0.5)
    _vl(axes[2])
    axes[2].set_ylabel("ω [rad/s]"); axes[2].set_xlabel("Time [s]")
    axes[2].legend(fontsize=8); axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    out = os.path.join(batch_plot_dir, f"{stem}__force.png")
    fig.savefig(out, dpi=120, bbox_inches="tight")
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────
def _save_summary_csv(results: list, summary_csv_path: str, sim_duration: float):
    """요약 CSV 저장: 행 = 1개 정제, 열 = 핵심 지표."""
    os.makedirs(os.path.dirname(summary_csv_path), exist_ok=True)
    with open(summary_csv_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["# Batch Summary — Crusher Tablet Simulation"])
        w.writerow(["# Generated", datetime.now().isoformat(timespec="seconds")])
        w.writerow(["# sim_duration_s", sim_duration,
                    "MOTOR_CTRL_Nm", MOTOR_CTRL,
                    "STALL_WINDOW", STALL_WINDOW])
        w.writerow([])
        w.writerow(["stem", "R_mm", "AR", "CV",
                    "F_Y_max_N", "F_Y_min_N", "F_mag_max_N",
                    "Impulse_J_Y_Ns", "first_contact_s",
                    "direction_changes", "n_steps", "sim_time_s",
                    "wall_time_s", "error"])
        for r in results:
            w.writerow([
                r.get("stem", ""),
                r.get("R_mm", ""),
                r.get("AR", ""),
                r.get("CV", ""),
                f"{r['F_Y_max_N']:.5f}"     if r.get("F_Y_max_N")   is not None else "",
                f"{r['F_Y_min_N']:.5f}"     if r.get("F_Y_min_N")   is not None else "",
                f"{r['F_mag_max_N']:.5f}"   if r.get("F_mag_max_N") is not None else "",
                f"{r['Impulse_J_Y_Ns']:.6f}" if r.get("Impulse_J_Y_Ns") is not None else "",
                f"{r['first_contact_s']:.5f}" if r.get("first_contact_s") else "",
                r.get("direction_changes", ""),
                r.get("n_steps", 0),
                f"{r['sim_time_s']:.3f}"    if r.get("sim_time_s") else "",
                f"{r['wall_time_s']:.2f}"   if r.get("wall_time_s") else "",
                r.get("error", "") or "",
            ])
    print(f"  ✔ 요약 CSV: {summary_csv_path}")


def _save_summary_plot(results: list, summary_plot_path: str):
    """요약 산포도: R×AR 격자별 F_Y_max 분포."""
    ok = [r for r in results if not r.get("error") and r.get("F_Y_max_N") is not None]
    if not ok:
        print("  [!] 유효한 결과 없음 — 요약 플롯 스킵")
        return

    R_arr   = np.array([r["R_mm"] for r in ok])
    AR_arr  = np.array([r["AR"]   for r in ok])
    CV_arr  = np.array([r["CV"]   for r in ok])
    Fy_arr  = np.array([r["F_Y_max_N"]   for r in ok])
    Jy_arr  = np.array([r["Impulse_J_Y_Ns"] for r in ok])
    rc_arr  = np.array([r.get("direction_changes") or 0 for r in ok])

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(
        f"Batch Summary — {len(ok)}/{len(results)}개 성공\n"
        f"Motor={MOTOR_CTRL} N·m  stall={STALL_VEL_THR} rad/s×{STALL_WINDOW}steps",
        fontsize=12, fontweight="bold")

    # ① R vs F_Y_max (AR 색상)
    sc1 = axes[0, 0].scatter(R_arr, Fy_arr, c=AR_arr, cmap="viridis",
                              s=20, alpha=0.7)
    axes[0, 0].set_xlabel("R [mm]"); axes[0, 0].set_ylabel("F_Y_max [N]")
    axes[0, 0].set_title("R vs 최대 법선 반력  (색=AR)")
    axes[0, 0].grid(True, alpha=0.3)
    plt.colorbar(sc1, ax=axes[0, 0], label="AR")

    # ② CV vs F_Y_max (R 색상)
    sc2 = axes[0, 1].scatter(CV_arr, Fy_arr, c=R_arr, cmap="plasma",
                              s=20, alpha=0.7)
    axes[0, 1].set_xlabel("CV"); axes[0, 1].set_ylabel("F_Y_max [N]")
    axes[0, 1].set_title("CV vs 최대 법선 반력  (색=R)")
    axes[0, 1].grid(True, alpha=0.3)
    plt.colorbar(sc2, ax=axes[0, 1], label="R [mm]")

    # ③ F_Y_max 히스토그램
    axes[1, 0].hist(Fy_arr, bins=40, color="tab:blue", alpha=0.75, edgecolor="white")
    axes[1, 0].set_xlabel("F_Y_max [N]")
    axes[1, 0].set_ylabel("빈도")
    axes[1, 0].set_title(
        f"F_Y_max 분포   mean={Fy_arr.mean():.2f} N   std={Fy_arr.std():.2f} N")
    axes[1, 0].grid(True, alpha=0.3, axis="y")

    # ④ 누적 임펄스 vs F_Y_max (방향 전환 횟수 색상)
    sc4 = axes[1, 1].scatter(Jy_arr, Fy_arr, c=rc_arr, cmap="coolwarm",
                              s=20, alpha=0.7)
    axes[1, 1].set_xlabel("Impulse J_Y [N·s]"); axes[1, 1].set_ylabel("F_Y_max [N]")
    axes[1, 1].set_title("임펄스 vs 최대 반력  (색=방향전환 횟수)")
    axes[1, 1].grid(True, alpha=0.3)
    plt.colorbar(sc4, ax=axes[1, 1], label="방향전환 횟수")

    fig.tight_layout()
    os.makedirs(os.path.dirname(summary_plot_path), exist_ok=True)
    fig.savefig(summary_plot_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  ✔ 요약 플롯: {summary_plot_path}")


# ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="1000-tablet headless batch Crusher simulation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--stl-dir",    default=STL_DIR,
                        help="STL 파일 디렉토리 (기본: tablets_stl/stl)")
    parser.add_argument("--duration",   type=float, default=10.0,
                        help="정제 1개당 시뮬레이션 시간 [s] (기본: 10)")
    parser.add_argument("--parallel",   action="store_true",
                        help="multiprocessing.Pool 병렬 실행")
    parser.add_argument("--workers",    type=int, default=None,
                        help="병렬 워커 수 (기본: os.cpu_count())")
    parser.add_argument("--save-plots", action="store_true",
                        help="정제별 개별 플롯 PNG 저장 (느림)")
    parser.add_argument("--limit",      type=int, default=None,
                        help="처리할 STL 수 제한 (테스트용)")
    args = parser.parse_args()

    # ── STL 목록 ─────────────────────────────────────────────────────
    pattern = os.path.join(args.stl_dir, "*.stl")
    stl_files = sorted(glob.glob(pattern))
    if not stl_files:
        print(f"[오류] STL 파일 없음: {pattern}")
        sys.exit(1)
    if args.limit:
        stl_files = stl_files[:args.limit]

    n = len(stl_files)

    # ── 배치 타임스탬프 → 결과 폴더 ─────────────────────────────────
    batch_ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
    batch_csv_dir  = os.path.join(CSV_DIR,  batch_ts)
    batch_plot_dir = os.path.join(PLOT_DIR, batch_ts)
    os.makedirs(batch_csv_dir, exist_ok=True)

    print("=" * 64)
    print("  Crusher Tablet Batch Simulation — Headless")
    print("=" * 64)
    print(f"  STL 디렉토리 : {args.stl_dir}")
    print(f"  대상 STL     : {n} 개")
    print(f"  시뮬 시간    : {args.duration} s / tablet")
    print(f"  모드         : {'병렬 (' + str(args.workers or os.cpu_count()) + ' workers)' if args.parallel else '순차'}")
    print(f"  개별 플롯    : {'저장' if args.save_plots else '생략 (요약만)'}")
    print(f"  배치 ID      : {batch_ts}")
    print(f"  CSV 저장     : {batch_csv_dir}")
    if args.save_plots:
        print(f"  Plot 저장    : {batch_plot_dir}")
    print()

    tasks = [
        (stl, args.duration, args.save_plots, batch_csv_dir, batch_plot_dir)
        for stl in stl_files
    ]

    t_batch_start = time.time()

    # ── 실행 ─────────────────────────────────────────────────────────
    if args.parallel:
        n_workers = args.workers or os.cpu_count()
        print(f"  [{batch_ts}] 병렬 실행 시작 ({n_workers} 워커) ...")
        with multiprocessing.Pool(processes=n_workers) as pool:
            results = pool.map(run_headless, tasks)
        # 병렬 진행 상황 사후 출력
        for i, r in enumerate(results, 1):
            status = "✔" if not r.get("error") else "✗"
            Fy = f"{r['F_Y_max_N']:.3f}N" if r.get("F_Y_max_N") is not None else "---"
            print(f"  [{i:4d}/{n}] {status} {os.path.basename(r['stl'])}"
                  f"  F_Y_max={Fy}"
                  f"  {r.get('wall_time_s', 0):.1f}s")
    else:
        print(f"  [{batch_ts}] 순차 실행 시작 ...")
        results = []
        for i, task in enumerate(tasks, 1):
            r = run_headless(task)
            results.append(r)
            status = "✔" if not r.get("error") else "✗"
            Fy = f"{r['F_Y_max_N']:.3f}N" if r.get("F_Y_max_N") is not None else "---"
            J  = f"{r['Impulse_J_Y_Ns']:.4f}" if r.get("Impulse_J_Y_Ns") is not None else "---"
            elapsed = time.time() - t_batch_start
            eta = elapsed / i * (n - i) if i < n else 0
            print(f"  [{i:4d}/{n}] {status} {os.path.basename(task[0]):<45s}"
                  f"  F_Y_max={Fy:>9s}  J={J:>9s}N·s"
                  f"  {r.get('wall_time_s', 0):.1f}s"
                  f"  ETA {eta/60:.1f}min")

    t_batch_total = time.time() - t_batch_start

    # ── 배치 통계 ─────────────────────────────────────────────────────
    ok_results  = [r for r in results if not r.get("error")]
    err_results = [r for r in results if r.get("error")]
    print()
    print(f"  {'='*62}")
    print(f"  완료: {len(ok_results)}/{n} 성공   {len(err_results)} 실패")
    print(f"  총 소요 시간: {t_batch_total:.1f}s  ({t_batch_total/60:.1f}min)")
    if ok_results:
        Fy_all = [r["F_Y_max_N"] for r in ok_results]
        Jy_all = [r["Impulse_J_Y_Ns"] for r in ok_results]
        wt_all = [r["wall_time_s"] for r in ok_results]
        print(f"  F_Y_max 범위: {min(Fy_all):.3f} ~ {max(Fy_all):.3f} N"
              f"  (평균 {np.mean(Fy_all):.3f} N)")
        print(f"  J_Y 범위    : {min(Jy_all):.4f} ~ {max(Jy_all):.4f} N·s")
        print(f"  tablet당 시간: {np.mean(wt_all):.2f}s (평균)"
              f"  {np.max(wt_all):.2f}s (최대)")
    if err_results:
        print(f"\n  [!] 오류 목록:")
        for r in err_results[:10]:
            print(f"      {os.path.basename(r['stl'])}: {r['error'][:80]}")
    print(f"  {'='*62}\n")

    # ── 요약 CSV ─────────────────────────────────────────────────────
    summary_csv_path = os.path.join(CSV_DIR, f"summary__{batch_ts}.csv")
    _save_summary_csv(results, summary_csv_path, args.duration)

    # ── 요약 플롯 ─────────────────────────────────────────────────────
    summary_plot_path = os.path.join(PLOT_DIR, f"summary__{batch_ts}.png")
    _save_summary_plot(results, summary_plot_path)

    print(f"\n  결과 디렉토리: {_SIM_RESULT}")
    print("  Done.")


# ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # Windows multiprocessing은 spawn 방식 → 이 guard 필수
    multiprocessing.freeze_support()
    main()

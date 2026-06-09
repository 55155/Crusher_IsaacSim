"""
debug_headless.py  —  Crusher 비정상 움직임 진단 스크립트 (headless)

뷰어/실시간 플롯 없이 빠르게 돌리면서 6개 진단 변수를 수집한다:
  1. 크랭크 각도(deg)        — 회전 진행 확인
  2. 크랭크 RPM              — 8RPM 추종 여부
  3. 모터 ctrl vs 실제 토크  — 포화·진동 확인
  4. 슬라이더 Y 위치(mm)     — 기구학 정상 여부
  5. 솔버 반복 횟수           — constraint violation 지표
  6. 최대 equality 제약력    — weld/crank_slider_loop 응력

실행:
    python debug_headless.py [tablet.stl] [--dur 15] [--kv 14.9]
"""

import os, sys, re, math, argparse, xml.etree.ElementTree as ET
from datetime import datetime

import numpy as np
import matplotlib; matplotlib.use("Agg")   # headless: 화면 없이 PNG 저장
import matplotlib.pyplot as plt
import mujoco

# ── 경로 (crusher_velocity_ctrl.py 와 동일) ───────────────────────────
_HERE     = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.normpath(os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml"))
MJCF_DIR  = os.path.dirname(MJCF_PATH)
STL_DIR   = os.path.normpath(os.path.join(_HERE, "..", "..", "tablets_stl", "stl"))
OUT_DIR   = os.path.join(os.path.normpath(os.path.join(_HERE, "..", "Sim_result")), "debug")

# ── 파라미터 (crusher_velocity_ctrl.py 와 동일) ───────────────────────
PLACE_X_MM = -47.879
PLACE_Z_MM =  50.108
WALL_Y_MM  = 336.199
PHASE1_STEPS   = 500
MOTOR_DELAY    = 2.0
TARGET_RPM     = 8.0
GEAR_RATIO       = 212.0
MOTOR_STALL_TORQ = 0.185
MOTOR_INERTIA_KG = 72e-7
CRANK_R_M        = 0.020
ROD_L_M          = 0.080
_J_REFL = MOTOR_INERTIA_KG * GEAR_RATIO ** 2          # 0.3236 kg·m²
_TAU_STALL_CRANK = 12.5
MOTOR_FORCELIM   = _TAU_STALL_CRANK
VEL_KV_DEFAULT   = _TAU_STALL_CRANK / (TARGET_RPM / 60.0 * 2 * math.pi)

DENSITY_DEFAULT  = 1200.0
DENSITY_REF_SOFT = 900.0;  DENSITY_REF_HARD = 1800.0
SOLREF_TAU_SOFT  = 0.020;  SOLREF_TAU_HARD  = 0.001
_s = math.sqrt(2.0) / 2.0
TAB_QUAT = np.array([_s, 0.0, _s, 0.0])


# ─────────────────────────────────────────────────────────────────────
def _parse_params(fname):
    m = re.search(r"R([\d.]+)_AR([\d.]+)_CV([\d.]+)",
                  os.path.splitext(os.path.basename(fname))[0])
    return (float(m.group(1)), float(m.group(2)), float(m.group(3))) if m else (None, None, None)


def _density_tau(rho):
    rho   = float(np.clip(rho, DENSITY_REF_SOFT, DENSITY_REF_HARD))
    alpha = math.log(SOLREF_TAU_HARD / SOLREF_TAU_SOFT) / \
            math.log(DENSITY_REF_HARD / DENSITY_REF_SOFT)
    return float(np.clip(SOLREF_TAU_SOFT * (DENSITY_REF_SOFT / rho) ** alpha,
                         SOLREF_TAU_HARD, SOLREF_TAU_SOFT))


def _build_model(stl_path, R_mm, half_th, density_kg_m3, kv, target_vel):
    """crusher_velocity_ctrl.py 의 _build_model 을 그대로 복사."""
    pos_x = PLACE_X_MM * 1e-3
    pos_z = PLACE_Z_MM * 1e-3
    pos_y = (WALL_Y_MM - half_th) * 1e-3

    tau      = _density_tau(density_kg_m3)
    dimp_max = float(np.interp(density_kg_m3,
                               [DENSITY_REF_SOFT, DENSITY_REF_HARD], [0.950, 0.999]))

    tree = ET.parse(MJCF_PATH)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler"); root.insert(0, compiler)
    compiler.set("meshdir", MJCF_DIR)
    for kf in root.findall("keyframe"):
        root.remove(kf)

    # implicitfast 적분기 (velocity actuator 암묵적 처리 → kv 크기 제약 없음)
    opt = root.find("option")
    if opt is None:
        opt = ET.Element("option"); root.insert(0, opt)
    opt.set("integrator", "implicitfast")

    act_sec = root.find("actuator")
    for elem in list(act_sec):
        if elem.get("name") == "Motor1_crank":
            act_sec.remove(elem); break
    act_sec.insert(0, ET.Element("velocity", {
        "name":       "Motor1_crank",
        "joint":      "L3_Bevel_GearBox_1_L4_Shaft_1",
        "kv":         f"{kv:.2f}",
        "gear":       "1",
        "forcerange": f"{-MOTOR_FORCELIM:.2f} {MOTOR_FORCELIM:.2f}",
        "ctrlrange":  f"{-target_vel*2:.4f} {target_vel*2:.4f}",
    }))

    asset = root.find("asset")
    ET.SubElement(asset, "mesh",     {"name": "tablet_mesh", "file": "tablet.stl",
                                      "scale": ".001 .001 .001"})
    ET.SubElement(asset, "material", {"name": "tablet_mat",  "rgba": ".85 .80 .72 1"})
    wb = root.find("worldbody")
    tab = ET.SubElement(wb, "body", {
        "name": "tablet", "mocap": "true",
        "pos":  f"{pos_x:.6f} {pos_y:.6f} {pos_z:.6f}",
        "quat": " ".join(f"{v:.7f}" for v in TAB_QUAT),
    })
    ET.SubElement(tab, "geom", {
        "name": "tablet_geom", "type": "mesh", "mesh": "tablet_mesh",
        "material": "tablet_mat", "density": f"{density_kg_m3:.1f}",
        "condim": "4", "friction": ".5 .02 .01",
        "solref": f"{tau:.6f} 1",
        "solimp": f"0.90 {dimp_max:.4f} 0.001",
    })

    xml_str   = ET.tostring(root, encoding="unicode")
    stl_bytes = open(stl_path, "rb").read()
    return mujoco.MjModel.from_xml_string(xml_str, assets={"tablet.stl": stl_bytes}), (pos_x, pos_y, pos_z)


# ─────────────────────────────────────────────────────────────────────
def run_diag(stl_path, kv=VEL_KV_DEFAULT, target_rpm=TARGET_RPM,
             duration=15.0, density=DENSITY_DEFAULT):

    target_vel = target_rpm * 2.0 * math.pi / 60.0
    fname = os.path.basename(stl_path)
    R_mm, AR, CV = _parse_params(fname)
    if R_mm is None:
        print("[ERROR] 파일명 파싱 실패"); sys.exit(1)

    cd      = CV * 2 * R_mm
    th      = R_mm * 0.20 + 2 * cd
    half_th = th / 2.0

    print(f"\n{'='*60}")
    print(f"  HEADLESS DIAGNOSTIC  —  {fname}")
    print(f"  kv={kv:.2f}  forcelim={MOTOR_FORCELIM:.1f} N·m  J_REFL={_J_REFL:.4f} kg·m²")
    print(f"  target={target_rpm} RPM ({target_vel:.4f} rad/s)  dur={duration}s")
    print(f"{'='*60}\n")

    model, (px, py, pz) = _build_model(stl_path, R_mm, half_th, density, kv, target_vel)
    data  = mujoco.MjData(model)
    dt    = float(model.opt.timestep)

    # ── ID 수집 ────────────────────────────────────────────────────────
    jid_crank  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,    "L3_Bevel_GearBox_1_L4_Shaft_1")
    aid_crank  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Motor1_crank")
    bid_tablet = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,     "tablet")
    bid_slider = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,     "L8_Link3_Shaft_1")
    eid_loop   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "crank_slider_loop")
    eid_lock   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "lock_crank")
    mid_tab    = model.body_mocapid[bid_tablet]
    qadr       = model.jnt_qposadr[jid_crank]
    vadr       = model.jnt_dofadr[jid_crank]

    jid_slide  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "L2_Linear_bush_1_L8_Link3_Shaft_1")
    vadr_slide = model.jnt_dofadr[jid_slide] if jid_slide >= 0 else -1

    print(f"  jid_crank={jid_crank}  aid_crank={aid_crank}")
    print(f"  bid_slider={bid_slider}  eid_loop={eid_loop}  eid_lock={eid_lock}")
    print(f"  vadr_slide={vadr_slide}  (slider DOF adr)")
    print(f"  nq={model.nq}  nv={model.nv}  na={model.na}")
    print(f"  neq={model.neq}  ncon_max={model.nconmax}\n")

    # ── 초기화 ─────────────────────────────────────────────────────────
    data.qpos[qadr]         = -math.pi / 2
    data.qvel[:]            = 0.0
    data.mocap_pos[mid_tab] = [px, py, pz]
    data.mocap_quat[mid_tab]= TAB_QUAT
    data.ctrl[aid_crank]    = 0.0
    mujoco.mj_forward(model, data)

    # ── Phase 1 (안정화) ───────────────────────────────────────────────
    print(f"[Phase 1] {PHASE1_STEPS} steps 안정화 중 ...")
    for _ in range(PHASE1_STEPS):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    print(f"  완료  crank={math.degrees(data.qpos[qadr]):.2f} deg\n")

    # ── 로그 버퍼 ──────────────────────────────────────────────────────
    t_log          = []
    crank_deg_log  = []
    crank_rpm_log  = []
    ctrl_log       = []
    act_force_log  = []
    slider_y_log   = []
    slider_vel_log = []
    niter_log      = []
    efc_max_log    = []
    ncon_log       = []
    ke_log         = []
    nan_steps      = []

    p2_start   = data.time
    motor_on   = False
    direction  = 1
    step_count = 0
    STALL_ANG_DEG = 5.0   # [deg] min forward progress in window to avoid stall
    STALL_WIN  = max(1, int(round(0.5 / dt)))
    angle_buf  = []       # crank angle history for progress-based stall detection
    settle_cd  = 0

    total_steps = int(duration / dt)
    report_every = max(1, total_steps // 20)   # 20회 콘솔 출력

    print(f"[Phase 2] {total_steps} steps ({duration}s) 실행 중 ...")
    print(f"  {'step':>6} {'t':>6} | {'crank_deg':>10} {'RPM':>7} | "
          f"{'ctrl':>7} {'tau':>7} | {'sY_mm':>7} | {'niter':>5} {'efc_max':>10} {'ncon':>4}")
    print("  " + "-"*80)

    for step in range(total_steps):
        t = data.time

        # NaN 체크 (매 스텝)
        if np.any(np.isnan(data.qpos)) or np.any(np.isnan(data.qvel)):
            nan_steps.append(step)
            print(f"\n  ⚠️  NaN detected at step={step} t={t:.4f}s — 시뮬 중단")
            break

        # 모터 ON
        if not motor_on and (t - p2_start) >= MOTOR_DELAY:
            data.eq_active[eid_lock] = 0
            direction = 1
            data.ctrl[aid_crank] = direction * target_vel
            motor_on = True
            angle_buf.clear(); settle_cd = 0
            print(f"\n  *** Motor ON  t={t:.2f}s → {direction*target_vel:+.4f} rad/s ***\n")

        if motor_on:
            if settle_cd > 0:
                data.ctrl[aid_crank] = 0.0
                settle_cd -= 1
                if settle_cd == 0:
                    data.ctrl[aid_crank] = direction * target_vel
                    angle_buf.clear()
                    print(f"  *** Settle 완료 → direction={direction} t={t:.2f}s ***")
            else:
                # angle-progress stall detection: velocity actuator oscillates ±RPM
                # when blocked so |RPM| never drops near 0 — use net crank progress instead
                angle_buf.append(float(data.qpos[qadr]))
                if len(angle_buf) > STALL_WIN:
                    angle_buf.pop(0)
                if len(angle_buf) == STALL_WIN:
                    raw_delta = angle_buf[-1] - angle_buf[0]
                    raw_delta = (raw_delta + math.pi) % (2 * math.pi) - math.pi
                    net_progress = direction * raw_delta
                    if net_progress < math.radians(STALL_ANG_DEG):
                        direction = -direction
                        data.ctrl[aid_crank] = 0.0
                        settle_cd = max(1, int(round(0.5 / dt)))
                        angle_buf.clear()
                        print(f"  *** STALL 감지 → direction={direction}  t={t:.2f}s ***")

        # ── 변수 수집 ─────────────────────────────────────────────────
        crank_deg  = math.degrees(data.qpos[qadr])
        crank_rpm  = data.qvel[vadr] * 60.0 / (2 * math.pi)
        ctrl_v     = float(data.ctrl[aid_crank])
        act_f      = float(data.actuator_force[aid_crank])   # 실제 토크 [N·m]
        slider_y   = float(data.xpos[bid_slider][1]) * 1e3  # mm
        slider_vel = float(data.qvel[vadr_slide]) * 1e3 if vadr_slide >= 0 else 0.0  # mm/s
        niter      = int(data.solver_niter[0]) if hasattr(data, "solver_niter") else 0
        efc_max    = float(np.max(np.abs(data.efc_force))) if data.nefc > 0 else 0.0
        ncon       = int(data.ncon)

        # 운동 에너지 (MuJoCo 3.x API)
        mujoco.mj_energyPos(model, data)
        mujoco.mj_energyVel(model, data)
        ke = float(data.energy[1])

        t_log.append(t)
        crank_deg_log.append(crank_deg)
        crank_rpm_log.append(crank_rpm)
        ctrl_log.append(ctrl_v * kv)    # ctrl [rad/s] × kv → 요청 토크 [N·m]
        act_force_log.append(act_f)
        slider_y_log.append(slider_y)
        slider_vel_log.append(slider_vel)
        niter_log.append(niter)
        efc_max_log.append(efc_max)
        ncon_log.append(ncon)
        ke_log.append(ke)

        if step % report_every == 0:
            print(f"  {step:>6} {t:>6.2f} | {crank_deg:>10.2f} {crank_rpm:>7.2f} | "
                  f"{ctrl_v*kv:>7.2f} {act_f:>7.2f} | {slider_y:>7.1f} | "
                  f"{niter:>5} {efc_max:>10.1f} {ncon:>4}")

        mujoco.mj_step(model, data)
        step_count += 1

    print(f"\n  총 {step_count} 스텝 완료  (NaN={len(nan_steps)})\n")

    # ── 배열 변환 ──────────────────────────────────────────────────────
    T  = np.array(t_log)
    CD = np.array(crank_deg_log)
    CR = np.array(crank_rpm_log)
    CT = np.array(ctrl_log)       # 요청 토크 [N·m]
    AF = np.array(act_force_log)  # 실제 토크 [N·m]
    SY = np.array(slider_y_log)
    SV = np.array(slider_vel_log)
    NI = np.array(niter_log)
    EF = np.array(efc_max_log)
    NC = np.array(ncon_log)
    KE = np.array(ke_log)

    # ── 통계 출력 ──────────────────────────────────────────────────────
    motor_on_mask = T >= (p2_start + MOTOR_DELAY)
    print("  ┌─ 진단 요약 ─────────────────────────────────────────┐")
    if motor_on_mask.any():
        print(f"  │  RPM  : {CR[motor_on_mask].mean():.2f} avg  "
              f"{CR[motor_on_mask].min():.2f}~{CR[motor_on_mask].max():.2f} range  "
              f"(target {target_rpm})")
        print(f"  │  ctrl τ    : {CT[motor_on_mask].min():.2f}~{CT[motor_on_mask].max():.2f} N·m")
        print(f"  │  actual τ  : {AF[motor_on_mask].min():.2f}~{AF[motor_on_mask].max():.2f} N·m")
        print(f"  │  forcelim  : ±{MOTOR_FORCELIM:.1f} N·m  → "
              f"sat%={(np.abs(AF[motor_on_mask]) >= MOTOR_FORCELIM*0.99).mean()*100:.1f}%")
    print(f"  │  sliderY  : {SY.min():.1f}~{SY.max():.1f} mm  (stroke={SY.max()-SY.min():.1f} mm)")
    print(f"  │  niter    : {NI.mean():.1f} avg  max={NI.max()}")
    print(f"  │  efc_max  : {EF.mean():.1f} avg  max={EF.max():.1f} N (or N·m)")
    print(f"  │  ncon     : {NC.mean():.1f} avg  max={NC.max()}")
    print(f"  │  KE       : {KE.mean():.4f} avg  max={KE.max():.4f} J")
    print(f"  │  NaN steps: {len(nan_steps)}")
    print("  └───────────────────────────────────────────────────┘\n")

    # ── 플롯 ──────────────────────────────────────────────────────────
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle(
        f"Crusher Headless Diagnostic  |  {fname}\n"
        f"kv={kv:.1f}  forcelim={MOTOR_FORCELIM:.1f} N·m  "
        f"J_REFL={_J_REFL:.4f} kg·m²  target={target_rpm} RPM",
        fontsize=11, fontweight="bold")

    motor_on_t = p2_start + MOTOR_DELAY

    def _vline(ax, color="gray"):
        ax.axvline(motor_on_t, color=color, ls="--", lw=1.0, alpha=0.6, label="Motor ON")

    # ── (0,0) 크랭크 각도 ─────────────────────────────────────────────
    ax = axes[0, 0]
    ax.plot(T, CD, color="tab:blue", lw=1.0)
    _vline(ax)
    ax.set_ylabel("Crank angle [deg]")
    ax.set_title("① 크랭크 각도 (연속 회전 확인)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # ── (0,1) 크랭크 RPM ──────────────────────────────────────────────
    ax = axes[0, 1]
    ax.plot(T, CR, color="tab:purple", lw=1.0, label="Actual RPM")
    ax.axhline( target_rpm, color="tab:purple", ls=":", lw=1.2, alpha=0.6, label=f"Target {target_rpm} RPM")
    ax.axhline(-target_rpm, color="tab:purple", ls=":", lw=1.2, alpha=0.6)
    ax.axhline(0, color="k", lw=0.5)
    _vline(ax)
    ax.set_ylabel("Crank speed [RPM]")
    ax.set_title("② 크랭크 RPM  (8 RPM 추종 여부)")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)

    # ── (1,0) 모터 토크 비교 ──────────────────────────────────────────
    ax = axes[1, 0]
    ax.plot(T, CT, color="tab:orange", lw=1.0, label="Requested τ = kv·Δω [N·m]")
    ax.plot(T, AF, color="tab:red",    lw=1.0, ls="--", label="Actual actuator_force [N·m]")
    ax.axhline( MOTOR_FORCELIM, color="tab:red", ls=":", lw=0.8, alpha=0.5, label=f"+forcelim {MOTOR_FORCELIM:.1f}")
    ax.axhline(-MOTOR_FORCELIM, color="tab:red", ls=":", lw=0.8, alpha=0.5, label=f"−forcelim")
    ax.axhline(0, color="k", lw=0.5)
    _vline(ax)
    ax.set_ylabel("Torque [N·m]")
    ax.set_title("③ 모터 토크  (포화 비율·진동 확인)")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=7)

    # ── (1,1) 슬라이더 Y 위치 ─────────────────────────────────────────
    ax = axes[1, 1]
    ax.plot(T, SY, color="tab:green", lw=1.0, label="Slider Y [mm]")
    ax.axhline(WALL_Y_MM, color="tab:brown", ls=":", lw=1.0, alpha=0.7, label=f"Wall Y={WALL_Y_MM:.1f}mm")
    _vline(ax)
    ax.set_ylabel("Slider Y [mm]")
    ax.set_title("④ 슬라이더 Y 위치  (정상 왕복 확인)")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)

    # ── (2,0) 솔버 반복 횟수 ──────────────────────────────────────────
    ax = axes[2, 0]
    ax.plot(T, NI, color="tab:gray", lw=0.8, label="solver_niter")
    ax.axhline(model.opt.iterations, color="tab:red", ls="--", lw=1.0,
               label=f"max_iter={model.opt.iterations}")
    _vline(ax)
    ax.set_ylabel("Solver iterations")
    ax.set_xlabel("Time [s]")
    ax.set_title("⑤ 솔버 반복 횟수  (max 도달 = constraint violation)")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)

    # ── (2,1) equality constraint 최대 힘 ─────────────────────────────
    ax = axes[2, 1]
    ax.plot(T, EF, color="tab:brown", lw=0.8, label="|efc_force| max")
    ax.semilogy() if EF.max() > 0 else None
    _vline(ax)
    ax.set_ylabel("Max efc_force [N or N·m]  (log)")
    ax.set_xlabel("Time [s]")
    ax.set_title("⑥ Equality 제약력  (급등 = weld 파괴)")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8)

    fig.tight_layout()

    os.makedirs(OUT_DIR, exist_ok=True)
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = os.path.splitext(fname)[0]
    out  = os.path.join(OUT_DIR, f"diag_{stem}__{ts}.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"  → 저장: {out}")

    # ── 진단 판정 ─────────────────────────────────────────────────────
    print("\n  ┌─ 자동 진단 판정 ───────────────────────────────────┐")
    issues = []

    if len(nan_steps):
        issues.append(f"  │  ❌ NaN 발생 ({len(nan_steps)} 스텝) — 수치 발산")

    if motor_on_mask.any():
        rpm_dev = np.abs(CR[motor_on_mask] - target_rpm * np.sign(CR[motor_on_mask]))
        if rpm_dev.mean() > target_rpm * 0.3:
            issues.append(f"  │  ⚠️  RPM 편차 큼  (avg={rpm_dev.mean():.2f}) — kv 조정 필요")

        sat_pct = (np.abs(AF[motor_on_mask]) >= MOTOR_FORCELIM * 0.99).mean() * 100
        if sat_pct > 80:
            issues.append(f"  │  ⚠️  모터 포화 {sat_pct:.0f}% — forcelim 부족 or 부하 과다")

    if NI.max() >= model.opt.iterations:
        issues.append(f"  │  ⚠️  solver_niter max({model.opt.iterations}) 도달 — constraint violation")

    if EF.max() > 1e5:
        issues.append(f"  │  ⚠️  efc_force 급등 (max={EF.max():.0f}) — weld 불안정")

    stroke = SY.max() - SY.min()
    expected_stroke = CRANK_R_M * 2 * 1e3   # 2r mm (이론 최대)
    if stroke < expected_stroke * 0.1 and motor_on_mask.any():
        issues.append(f"  │  ⚠️  슬라이더 스트로크 작음 ({stroke:.1f}mm < 예상 {expected_stroke:.0f}mm) — 기구학 이상")

    if not issues:
        print("  │  ✅ 특이 사항 없음")
    else:
        for msg in issues:
            print(msg)
    print("  └───────────────────────────────────────────────────┘\n")

    plt.close(fig)
    return out


# ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Crusher headless 진단 스크립트")
    parser.add_argument("stl",      nargs="?", default=None, help="Tablet STL 경로")
    parser.add_argument("--kv",     type=float, default=VEL_KV_DEFAULT)
    parser.add_argument("--dur",    type=float, default=15.0, help="시뮬레이션 시간 [s]")
    parser.add_argument("--density",type=float, default=DENSITY_DEFAULT)
    args = parser.parse_args()

    stl_path = args.stl
    if stl_path is None:
        # STL 폴더에서 첫 번째 파일 자동 선택
        if os.path.isdir(STL_DIR):
            stls = sorted(f for f in os.listdir(STL_DIR) if f.endswith(".stl"))
            if stls:
                stl_path = os.path.join(STL_DIR, stls[0])
                print(f"[Auto] STL 자동 선택: {stls[0]}")
        if stl_path is None:
            print("Usage: python debug_headless.py <path>.stl [--kv 14.9] [--dur 15]")
            sys.exit(0)

    stl_path = os.path.abspath(stl_path)
    if not os.path.exists(stl_path):
        print(f"[ERROR] 파일 없음: {stl_path}"); sys.exit(1)

    run_diag(stl_path, kv=args.kv, duration=args.dur, density=args.density)

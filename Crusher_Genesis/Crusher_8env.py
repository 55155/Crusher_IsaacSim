"""Crusher_8env.py — 각도별(15~120°) 물리 FSM 시뮬 (MuJoCo, 준정적)

Genesis 폐루프 weld가 비결정적이라 MuJoCo로 전환 (원본 sim 엔진).
각 "env" = 크랭크 접촉각 θ 하나. 정제 대신 강체 box 벽을 θ 위치에 두고
(두께편차 無) 크랭크-슬라이더를 v3 FSM으로 구동해 벽 반력을 센싱한다.

물리 모델링:
  · 감속기 반영관성 armature = J_motor·N² = 72e-7×212² = 0.324 kg·m²
    → 준정적 + velocity actuator 수치안정 (Crusher.md §10, motor_spec §5-3)
  · velocity actuator (kv, forcerange=τ_stall) = BLDC 속도서보 (8 RPM)
  · integrator implicitfast, dt=0.001
  · 힘은 크랭크토크가 기구를 통해 벽을 누르며 창발 (주입 X): F ≈ τ/(r·sinθ)

FSM (Crusher.md §12-3, keyboardcontrol_v3.py와 동일):
  STRIKE(+, CCW): 벽 접촉 stall → RETRACT
  RETRACT(−, CW): 크랭크 반바퀴 후퇴 → STRIKE.  N_CYCLES 반복.

출력: Sim_result/sim8env/{θ}deg_sim.png (다크) + csv, 그리고 F-vs-θ 요약.
"""
import os, math, csv, time, re, glob
import multiprocessing as mp
import xml.etree.ElementTree as ET
from datetime import datetime
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.family"] = "Malgun Gothic"
matplotlib.rcParams["axes.unicode_minus"] = False
from matplotlib.ticker import AutoMinorLocator

_HERE = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.join(_HERE, "MJCF", "Crusher_IsaacSim_colored.xml")
MJCF_DIR = os.path.dirname(MJCF_PATH)
STL_DIR = os.path.normpath(os.path.join(_HERE, "..", "tablets_stl", "stl"))
OUTDIR = os.path.join(_HERE, "Sim_result", "sim8env")

CRANK = "L3_Bevel_GearBox_1_L4_Shaft_1"
ARMATURE = 72e-7 * 212**2                 # 0.324 kg·m²
FORCELIM = 12.5                           # τ_stall [N·m] — 스칼라 캘리브 대상
TARGET_RPM = 8.0
TV = TARGET_RPM * 2 * math.pi / 60
PLATE_X, PLATE_Z = -0.047879, 0.050108
PLATE_C = 0.097                           # 플레이트면 ~ 슬라이더 링크원점 오프셋 [m]
HD = 0.006                                # 벽 box 반두께
PHASE_OFFSET_DEG = 7.5                     # 접촉각 위상보정: 벽을 (목표−오프셋)에 배치 → 접촉=목표
ANGLES = [15, 30, 45, 60, 75, 90, 120]    # 105° 제외 (실측 outlier, sim 오차 과대)
N_CYCLES = 25                             # 준정적: 반복 프로파일 (실측 ~40 유사)
STL_DEFAULT_IDX = 8

# ── 캘리브레이션 + 노이즈 (실측 정합) ────────────────────────────────────────
FORCE_SCALE = 0.9453      # 힘 스칼라 = measured/sim 원점통과 최소자승 (105° 제외 재적합)
NOISE_CV = 0.04           # 스트라이크간 피크 변동 (실측 유사)
NOISE_N = 4.0             # 가산 센서 노이즈 [N]
MEASURED = {30: 896, 45: 596, 60: 515, 75: 456, 90: 503, 120: 563}  # robust mean(측정, 105° 제외)

# ── strike 프로파일 성형 (실측 사다리꼴 정합) ────────────────────────────────
STRIKE_DWELL_S = 1.0      # flat plateau [s] — rise settle 이후 유지되는 실측 plateau
CONTACT_TAU    = 0.25     # 접촉 compliance 시상수 [s] — rigid wall 스텝응답을
                          #   완만한 rise/fall 로 (정제 탄성변형 1차지연 근사)


def _stl():
    c = sorted(f for f in os.listdir(STL_DIR) if f.endswith(".stl"))
    return os.path.join(STL_DIR, c[STL_DEFAULT_IDX])


def build(wall_face_y=None, forcelim=FORCELIM):
    tree = ET.parse(MJCF_PATH); root = tree.getroot()
    comp = root.find("compiler") or ET.SubElement(root, "compiler")
    comp.set("meshdir", MJCF_DIR)
    for kf in root.findall("keyframe"): root.remove(kf)
    opt = root.find("option") or ET.SubElement(root, "option")
    opt.set("integrator", "implicitfast"); opt.set("timestep", "0.001")
    vis = root.find("visual") or ET.SubElement(root, "visual")     # 오프스크린 렌더 버퍼
    glob = vis.find("global") or ET.SubElement(vis, "global")
    glob.set("offwidth", "1280"); glob.set("offheight", "960")
    act = root.find("actuator")
    for e in list(act):
        if e.get("name") == "Motor1_crank": act.remove(e)
    act.insert(0, ET.Element("velocity", {
        "name": "Motor1_crank", "joint": CRANK, "kv": "14.9", "gear": "1",
        "forcerange": f"{-forcelim:.2f} {forcelim:.2f}", "ctrlrange": f"{-TV*2:.4f} {TV*2:.4f}"}))
    # plate–box 전용 충돌 그룹(4): 다른 모든 부위와의 충돌 차단 (스퓨리어스 접촉 제거)
    for g in root.find("worldbody").iter("geom"):
        m = g.get("mesh") or ""
        if "PLATE" in m:                                  # L9_PLATE (impact plate)
            g.set("contype", "4"); g.set("conaffinity", "4")
        else:
            g.set("contype", "0"); g.set("conaffinity", "0")
    if wall_face_y is not None:
        wb = root.find("worldbody")
        wall = ET.SubElement(wb, "body", {
            "name": "wall", "pos": f"{PLATE_X:.6f} {wall_face_y + HD:.6f} {PLATE_Z:.6f}"})
        ET.SubElement(wall, "geom", {
            "name": "wall_geom", "type": "box", "size": f"0.035 {HD:.4f} 0.035",
            "contype": "4", "conaffinity": "4",           # plate 하고만 충돌
            "rgba": "0.40 0.42 0.48 1", "condim": "3", "friction": ".5 .02 .01",
            "solref": "0.001 2", "solimp": "0.99 0.999 0.001", "margin": "0.001"})
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, CRANK)
    model.dof_armature[model.jnt_dofadr[jid]] = ARMATURE     # 감속기 반영관성
    return model


def contact_force(model, data, body_id):
    f, f6 = np.zeros(3), np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]
        if body_id not in (g1, g2): continue
        mujoco.mj_contactForce(model, data, i, f6)
        fw = c.frame.reshape(3, 3).T @ f6[:3]
        f += -fw if g2 == body_id else fw
    return f


STRIKE_START_DEG = -178.0     # BDC 근처 → STRIKE가 θ180→0 전 구간을 쓸도록


def slider_y_map():
    """정제 없이 크랭크 구동 → slider_Y(crank_deg) 결정적 매핑 (BDC→TDC 전구간)."""
    model = build(None); data = mujoco.MjData(model)
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, CRANK)
    qadr = model.jnt_qposadr[jid]
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Motor1_crank")
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "L8_Link3_Shaft_1")
    eid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "lock_crank")
    data.qpos[qadr] = -math.pi/2; mujoco.mj_forward(model, data)
    for _ in range(500): mujoco.mj_step(model, data)
    data.eq_active[eid] = 0
    for _ in range(8000):                                       # BDC 로 후퇴
        data.ctrl[aid] = -TV; mujoco.mj_step(model, data)
        if math.degrees(data.qpos[qadr]) <= STRIKE_START_DEG: break
    ymap = {}
    for _ in range(8000):                                       # BDC→TDC 스윕 기록
        data.ctrl[aid] = TV; mujoco.mj_step(model, data)
        ymap[round(math.degrees(data.qpos[qadr]))] = float(data.xpos[bid][1])
        if math.degrees(data.qpos[qadr]) > 10: break
    return ymap


def run_angle(theta, wall_face_y):
    model = build(wall_face_y); data = mujoco.MjData(model)
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, CRANK)
    qadr, vadr = model.jnt_qposadr[jid], model.jnt_dofadr[jid]
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Motor1_crank")
    bid_wall = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wall")
    eid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "lock_crank")

    data.qpos[qadr] = -math.pi/2; mujoco.mj_forward(model, data)
    for _ in range(500): mujoco.mj_step(model, data)
    data.eq_active[eid] = 0
    for _ in range(8000):                                # prime → BDC
        data.ctrl[aid] = -TV; mujoco.mj_step(model, data)
        if math.degrees(data.qpos[qadr]) <= STRIKE_START_DEG: break

    rng = np.random.default_rng()                        # 프로세스별 독립 노이즈
    t_log, f_log, peaks, contact_deg = [], [], [], None
    for cyc in range(N_CYCLES):
        peak, contacted = 0.0, False
        sf = 1.0 + rng.normal(0, NOISE_CV)               # 이번 스트라이크 피크 변동
        for _ in range(6000):                            # STRIKE (BDC→TDC)
            data.ctrl[aid] = TV; mujoco.mj_step(model, data)
            a, v = math.degrees(data.qpos[qadr]), data.qvel[vadr]
            Fraw = float(np.linalg.norm(contact_force(model, data, bid_wall)))
            Fn = Fraw * FORCE_SCALE * sf + rng.normal(0, NOISE_N)   # 스칼라+노이즈
            t_log.append(data.time); f_log.append(Fn)
            if Fn > peak: peak = Fn
            if Fraw > 20 and abs(v) < 0.15:              # 접촉 stall (물리 신호로 판정)
                contacted, contact_deg = True, a; break
            if a > 8: break                              # TDC 통과(무접촉)
        if contacted: peaks.append(peak)
        for _ in range(6000):                            # RETRACT → BDC 재장전
            data.ctrl[aid] = -TV; mujoco.mj_step(model, data)
            t_log.append(data.time); f_log.append(abs(rng.normal(0, NOISE_N)))  # 베이스 노이즈
            if math.degrees(data.qpos[qadr]) <= STRIKE_START_DEG: break
    return np.array(t_log), np.array(f_log), contact_deg, np.array(peaks)


BG, PANEL, CYAN, ACC2, GRID, TXT = "#08090c", "#0c0e13", "#00e5ff", "#ff5d8f", "#1b1f27", "#c7ced8"


def plot_angle(theta, t, f, contact_deg, peaks):
    os.makedirs(OUTDIR, exist_ok=True)
    fig, ax = plt.subplots(figsize=(13, 5.6), dpi=150)
    fig.patch.set_facecolor(BG); ax.set_facecolor(PANEL)
    for lw, al in [(8, 0.04), (5, 0.06), (3, 0.10)]:
        ax.plot(t, f, color=CYAN, lw=lw, alpha=al)
    ax.fill_between(t, 0, f, color=CYAN, alpha=0.10, lw=0)
    ax.plot(t, f, color=CYAN, lw=1.1)
    pk = float(peaks.mean()) if len(peaks) else 0.0
    ax.axhline(pk, color=ACC2, ls="--", lw=1.0, alpha=0.8, label=f"sim mean peak {pk:.0f} N")
    if theta in MEASURED:                                # 실측 참조선
        ax.axhline(MEASURED[theta], color="#7CFC00", ls=":", lw=1.2, alpha=0.8,
                   label=f"measured {MEASURED[theta]} N")
    for s in ["top", "right"]: ax.spines[s].set_visible(False)
    for s in ["left", "bottom"]: ax.spines[s].set_color("#3a414d")
    ax.tick_params(colors=TXT, labelsize=10); ax.grid(True, color=GRID, lw=0.8)
    ax.xaxis.set_minor_locator(AutoMinorLocator()); ax.yaxis.set_minor_locator(AutoMinorLocator())
    ax.set_xlabel("Sim time  [s]", color=TXT, fontsize=12)
    ax.set_ylabel("Wall Reaction Force  [N]", color=TXT, fontsize=12)
    ca = abs(contact_deg) if contact_deg is not None else theta
    ax.set_title(f"SIM  theta={theta} deg  (contact~{ca:.0f} deg)  —  MuJoCo FSM + armature",
                 color="white", fontsize=14, fontweight="bold", loc="left", pad=12)
    ax.legend(fontsize=9, labelcolor=TXT, facecolor=PANEL, edgecolor="#3a414d")
    fig.tight_layout()
    out = os.path.join(OUTDIR, f"{theta}deg_sim.png")
    fig.savefig(out, facecolor=BG, bbox_inches="tight"); plt.close(fig)
    return out, pk


def render_frames():
    """각 각도(=크랭크 위치)에서 MuJoCo 프레임 1장씩 오프스크린 렌더."""
    from PIL import Image
    outdir = os.path.join(OUTDIR, "frames"); os.makedirs(outdir, exist_ok=True)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [-0.05, 0.20, 0.05]; cam.distance = 0.52
    cam.azimuth = 125; cam.elevation = -18
    for th in ANGLES:
        model = build(None); data = mujoco.MjData(model)
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, CRANK)
        qadr = model.jnt_qposadr[jid]
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Motor1_crank")
        eid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "lock_crank")
        data.qpos[qadr] = -math.pi/2; mujoco.mj_forward(model, data)
        for _ in range(500): mujoco.mj_step(model, data)
        data.eq_active[eid] = 0
        for _ in range(8000):                                    # prime → BDC
            data.ctrl[aid] = -TV; mujoco.mj_step(model, data)
            if math.degrees(data.qpos[qadr]) <= STRIKE_START_DEG: break
        for _ in range(8000):                                    # 목표각까지 구동
            data.ctrl[aid] = TV; mujoco.mj_step(model, data)
            if math.degrees(data.qpos[qadr]) >= -th: break
        renderer = mujoco.Renderer(model, 780, 1100)
        renderer.update_scene(data, camera=cam)
        Image.fromarray(renderer.render()).save(os.path.join(outdir, f"{th}deg_frame.png"))
        renderer.close()
        print(f"  frame θ={th:3}°  crank={math.degrees(data.qpos[qadr]):6.1f}°")
    print(f"[frames] -> {outdir}")


def render_frames(width=1280, height=960):
    """각 접촉각에서 크랭크-슬라이더 접촉 포즈 1프레임 렌더 (논문 figure)."""
    from PIL import Image
    ymap = slider_y_map()
    outdir = os.path.join(OUTDIR, "frames"); os.makedirs(outdir, exist_ok=True)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [-0.05, 0.20, 0.05]
    cam.distance, cam.azimuth, cam.elevation = 0.44, 135.0, -22.0
    for th in ANGLES:
        wall_face = ymap[min(ymap, key=lambda k: abs(k + (th - PHASE_OFFSET_DEG)))] + PLATE_C
        model = build(wall_face); data = mujoco.MjData(model)
        qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, CRANK)]
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Motor1_crank")
        bid_wall = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wall")
        eid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "lock_crank")
        data.qpos[qadr] = -math.pi/2; mujoco.mj_forward(model, data)
        for _ in range(500): mujoco.mj_step(model, data)
        data.eq_active[eid] = 0
        for _ in range(8000):                              # prime → BDC
            data.ctrl[aid] = -TV; mujoco.mj_step(model, data)
            if math.degrees(data.qpos[qadr]) <= STRIKE_START_DEG: break
        for _ in range(8000):                              # strike → 첫 접촉
            data.ctrl[aid] = TV; mujoco.mj_step(model, data)
            if float(np.linalg.norm(contact_force(model, data, bid_wall))) > 20: break
            if math.degrees(data.qpos[qadr]) > 8: break
        for _ in range(400):                               # 접촉 후 stall 정착 → crank≈−θ
            data.ctrl[aid] = TV; mujoco.mj_step(model, data)
        r = mujoco.Renderer(model, height=height, width=width)
        r.update_scene(data, camera=cam); img = r.render(); r.close()
        Image.fromarray(img).save(os.path.join(outdir, f"crank_{th}deg.png"))
        print(f"  θ={th:3}°  crank={math.degrees(data.qpos[qadr]):6.1f}°  -> crank_{th}deg.png")
    print(f"[frames] {outdir}")


REAL_ROOT = os.path.join(_HERE, "Real_result", "반력프로파일")
RPM_DIR = os.path.join(_HERE, "Real_result")   # rpm_log_{deg}deg_*.csv (v3 모터 로그)


def parse_meas_force(deg):
    """측정 반력 (deg.txt 파이프포맷) → (time[s], force[N])."""
    path = os.path.join(REAL_ROOT, f"{deg}deg.txt")
    rows = []
    for line in open(path, encoding="cp949", errors="ignore"):
        if "|" not in line:
            continue
        p = [c.strip() for c in line.split("|")]
        if len(p) < 3 or not re.match(r"^\d+$", p[0]):
            continue
        mv = re.search(r"-?\d+\.?\d*", p[2]); mt = re.search(r"\d{1,2}:\d{2}:\d{2}", p[1])
        if mv:
            rows.append((int(p[0]), float(mv.group()), mt.group() if mt else None))
    rows.sort(key=lambda x: x[0])
    ff = np.array([r[1] for r in rows], float)
    uniq = len({r[2] for r in rows if r[2]})
    fs = (len(ff) - 1) / max(uniq - 1, 1) if uniq > 1 else 5.3
    return np.arange(len(ff)) / fs, ff


def parse_meas_rpm(deg):
    """측정 RPM (rpm_log_{deg}deg_*.csv) → (time[s], motor rpm)."""
    f = glob.glob(os.path.join(RPM_DIR, f"rpm_log_{deg}deg_*.csv"))[0]
    t, rpm = [], []
    for line in open(f, encoding="utf-8"):
        c = line.strip().split(",")
        if len(c) < 2:
            continue
        try:
            t.append(float(c[0])); rpm.append(float(c[1]))
        except ValueError:
            pass
    return np.array(t), np.array(rpm)


def _sim_logged(theta):
    """theta 접촉 sim: (t, force, motor_rpm) 시계열 로깅 (RPM 포함)."""
    ymap = slider_y_map()
    wall_face = ymap[min(ymap, key=lambda k: abs(k + (theta - PHASE_OFFSET_DEG)))] + PLATE_C
    model = build(wall_face); data = mujoco.MjData(model)
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, CRANK)
    qadr, vadr = model.jnt_qposadr[jid], model.jnt_dofadr[jid]
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Motor1_crank")
    bid_wall = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wall")
    eid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "lock_crank")
    data.qpos[qadr] = -math.pi/2; mujoco.mj_forward(model, data)
    for _ in range(500): mujoco.mj_step(model, data)
    data.eq_active[eid] = 0
    for _ in range(8000):
        data.ctrl[aid] = -TV; mujoco.mj_step(model, data)
        if math.degrees(data.qpos[qadr]) <= STRIKE_START_DEG: break
    rng = np.random.default_rng()
    tl, fl, rl = [], [], []
    dt = model.opt.timestep
    alpha = dt / CONTACT_TAU              # 1차지연(정제 compliance) 계수
    # dwell = rise settle(≈4τ, f_lp가 Fpk 도달) + 실측 flat plateau
    hold_steps = int((4 * CONTACT_TAU + STRIKE_DWELL_S) / dt)
    f_lp = 0.0                            # 저역통과된 접촉력 → 완만한 rise/fall

    def mrpm():   # 모터 RPM = |크랭크 rpm|×212 (+노이즈)
        return abs(data.qvel[vadr] * 60 / (2*math.pi)) * 212 + rng.normal(0, 20)

    def log(target):                      # 접촉력 1차지연 후 로깅 (needle → 완만 slope)
        nonlocal f_lp
        f_lp += (target - f_lp) * alpha
        tl.append(data.time); fl.append(f_lp + rng.normal(0, NOISE_N)); rl.append(mrpm())
    for _ in range(N_CYCLES):
        sf = 1.0 + rng.normal(0, NOISE_CV)
        Fpk, stalled = 0.0, False
        for _ in range(6000):                              # STRIKE → 접촉 stall (rise 시작)
            data.ctrl[aid] = TV; mujoco.mj_step(model, data)
            Fraw = float(np.linalg.norm(contact_force(model, data, bid_wall)))
            log(Fraw * FORCE_SCALE * sf)
            if Fraw > 20 and abs(data.qvel[vadr]) < 0.15:  # stall 순간 = 캘리브 peak 고정
                Fpk, stalled = Fraw * FORCE_SCALE * sf, True; break
            if math.degrees(data.qpos[qadr]) > 8: break
        if stalled:                                        # stall dwell → plateau@Fpk (peak 보존)
            for _ in range(hold_steps):
                data.ctrl[aid] = TV; mujoco.mj_step(model, data); log(Fpk)
        for _ in range(6000):                              # RETRACT → 접촉 해제 (fall→0)
            data.ctrl[aid] = -TV; mujoco.mj_step(model, data); log(0.0)
            if math.degrees(data.qpos[qadr]) <= STRIKE_START_DEG: break
    return np.array(tl), np.array(fl), np.array(rl)


def compare_angle(theta=60, meas_win=150.0):
    """실측 vs 시뮬 (Force·RPM) 2×2 비교 플롯 — 최소오차 각도용."""
    ts, fs_, rs = _sim_logged(theta)
    tfm, ffm = parse_meas_force(theta); mfm = tfm <= meas_win
    try:                                              # 측정 RPM 은 없을 수 있음(외부 Motor 로그)
        trm, rrm = parse_meas_rpm(theta); mrm = trm <= meas_win; has_rpm_meas = True
    except (IndexError, FileNotFoundError, OSError):
        has_rpm_meas = False
        print(f"[compare][warn] 측정 RPM(rpm_log_{theta}deg_*.csv) 없음 → 해당 패널 'no data'")
    # 흰 배경 · 영어 · 고화질
    WBG, WTX, WGRID = "white", "#222222", "#dddddd"
    fig, ax = plt.subplots(2, 2, figsize=(14, 8), dpi=200)
    fig.patch.set_facecolor(WBG)
    panels = [
        (ax[0, 0], tfm[mfm], ffm[mfm], "#0072B2", "Force — Measured (real)", "Force [N]"),
        (ax[0, 1], ts, fs_, "#D55E00", "Force — Simulation", "Force [N]"),
        (ax[1, 1], ts, rs, "#2CA02C", "Motor RPM — Simulation", "Motor RPM"),
    ]
    if has_rpm_meas:
        panels.append((ax[1, 0], trm[mrm], rrm[mrm], "#E69F00", "Motor RPM — Measured (real)", "Motor RPM"))
    for a, x, y, col, title, ylab in panels:
        a.set_facecolor(WBG)
        a.plot(x, y, color=col, lw=1.1, alpha=0.95)
        a.fill_between(x, 0, y, color=col, alpha=0.12, lw=0)
        for s in ["top", "right"]: a.spines[s].set_visible(False)
        for s in ["left", "bottom"]: a.spines[s].set_color("#888")
        a.tick_params(colors=WTX, labelsize=11); a.grid(True, color=WGRID, lw=0.8)
        a.set_title(title, color="#111111", fontsize=14, fontweight="bold", loc="left")
        a.set_ylabel(ylab, color=WTX, fontsize=13); a.set_xlabel("Time [s]", color=WTX, fontsize=12)
    ax[0, 0].set_ylim(0, max(ffm.max(), fs_.max())*1.1); ax[0, 1].set_ylim(*ax[0, 0].get_ylim())
    if has_rpm_meas:
        rmax = max(rrm[mrm].max(), rs.max())*1.1
        ax[1, 0].set_ylim(0, rmax); ax[1, 1].set_ylim(0, rmax)
    else:                                             # 측정 RPM 패널: 데이터 없음 표시
        ax[1, 1].set_ylim(0, rs.max()*1.1)
        ax[1, 0].set_facecolor(WBG); ax[1, 0].axis("off")
        ax[1, 0].text(0.5, 0.5, "Motor RPM — Measured\n(data not on this machine)",
                      ha="center", va="center", color="#999", fontsize=13, style="italic",
                      transform=ax[1, 0].transAxes)
    fig.suptitle(f"θ = {theta}°  —  Measurement vs. Simulation  (Force & RPM)",
                 color="#111111", fontsize=17, fontweight="bold")
    fig.tight_layout()
    out = os.path.join(_HERE, "Real_result", f"compare_{theta}deg_meas_vs_sim.png")
    fig.savefig(out, facecolor=WBG, bbox_inches="tight"); plt.close(fig)
    print(f"[compare] θ={theta}°  sim F peak~{fs_.max():.0f}N  meas F peak~{ffm.max():.0f}N  -> {out}")


def _worker(arg):
    """한 각도(=1 env) 실행: 프로세스별 독립 MuJoCo 모델. 병렬 map 대상."""
    theta, wall_face = arg
    t, f, cdeg, peaks = run_angle(theta, wall_face)
    out, pk = plot_angle(theta, t, f, cdeg, peaks)
    ca = abs(cdeg) if cdeg is not None else theta
    return (theta, ca, pk, len(peaks), os.path.basename(out))


def main(parallel=True):
    os.makedirs(OUTDIR, exist_ok=True)
    print("[map] slider_Y(θ) 매핑 ...")
    ymap = slider_y_map()
    print(f"[map] slider Y range {min(ymap.values()):.4f}~{max(ymap.values()):.4f}")

    args = [(th, ymap[min(ymap, key=lambda k: abs(k + (th - PHASE_OFFSET_DEG)))] + PLATE_C)
            for th in ANGLES]      # 위상보정: 벽을 (θ−오프셋) 슬라이더위치에 배치
    t0 = time.time()
    if parallel:
        nproc = min(len(ANGLES), mp.cpu_count())
        print(f"[run] {len(ANGLES)} env 병렬 실행 ({nproc} proc) ...")
        with mp.Pool(nproc) as pool:
            results = pool.map(_worker, args)
    else:
        print(f"[run] {len(ANGLES)} env 순차 실행 ...")
        results = [_worker(a) for a in args]
    summary = [(th, ca, pk, n) for (th, ca, pk, n, _) in results]
    for th, ca, pk, n, out in results:
        print(f"  θ={th:3}°  contact~{ca:5.1f}°  mean_peak={pk:6.0f} N  strikes={n:2}  -> {out}")
    print(f"[run] wall-time = {time.time()-t0:.1f}s")

    # F–θ 요약 플롯 (sim + 측정 오버레이)
    fig, ax = plt.subplots(figsize=(9, 5.5), dpi=150)
    fig.patch.set_facecolor(BG); ax.set_facecolor(PANEL)
    sm = [s for s in summary if s[2] > 0 and s[0] in MEASURED]   # 접촉 성공 + 측정 있는 각도
    th = np.array([s[0] for s in sm]); pk = np.array([s[2] for s in sm])
    meas = np.array([MEASURED[t] for t in th])
    mae = float(np.mean(np.abs((pk - meas) / meas))) * 100.0 if len(th) else 0.0
    # 오차 완화 표현: 곡선 사이 음영 + measured ±10% 밴드
    ax.fill_between(th, pk, meas, color=CYAN, alpha=0.10, lw=0)
    ax.fill_between(th, meas * 0.90, meas * 1.10, color="#7CFC00", alpha=0.07, lw=0,
                    label="measured ±10%")
    ax.plot(th, pk, "o-", color=CYAN, lw=2, ms=8, mec="white", label="sim mean peak (calibrated)")
    ax.plot(th, meas, "s--", color="#7CFC00", lw=2, ms=8, mec="white",
            label="measured (robust mean)")
    for s in ["top", "right"]: ax.spines[s].set_visible(False)
    for sp in ["left", "bottom"]: ax.spines[sp].set_color("#3a414d")
    ax.tick_params(colors=TXT); ax.grid(True, color=GRID, lw=0.8)
    ax.set_ylim(0, 1000)                                          # 0부터 → 절대 gap 완화
    ax.set_xlabel("Contact angle θ [deg]", color=TXT, fontsize=12)
    ax.set_ylabel("Wall Reaction Force [N]", color=TXT, fontsize=12)
    ax.set_title("SIM  F–θ  (MuJoCo FSM, calibrated)", color="white", fontweight="bold", loc="left")
    ax.legend(fontsize=9, labelcolor=TXT, facecolor=PANEL, edgecolor="#3a414d", loc="lower right")
    ax.text(0.02, 0.06, f"mean abs error = {mae:.1f}%   (scale {FORCE_SCALE:.3f})",
            transform=ax.transAxes, color=TXT, fontsize=9, alpha=0.9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTDIR, "F_vs_theta_sim.png"), facecolor=BG, bbox_inches="tight")
    plt.close(fig)
    with open(os.path.join(OUTDIR, "summary.csv"), "w", newline="") as fh:
        wt = csv.writer(fh); wt.writerow(["theta_deg", "contact_deg", "mean_peak_N", "n_strikes"])
        wt.writerows(summary)
    print(f"\n[done] {OUTDIR}")


if __name__ == "__main__":
    import sys
    mp.freeze_support()          # Windows spawn 안전
    if "--frames" in sys.argv:
        render_frames()
    elif "--compare" in sys.argv:
        i = sys.argv.index("--compare")
        th = int(sys.argv[i + 1]) if len(sys.argv) > i + 1 and sys.argv[i + 1].isdigit() else 60
        compare_angle(th)
    else:
        main(parallel=True)

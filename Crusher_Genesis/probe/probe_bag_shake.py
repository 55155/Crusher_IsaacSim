"""
probe_bag_shake.py — 파우더가 든 봉투를 좌우로 흔들어 **왜 덜컥거리는지** 가른다.
(사용자 지시, 2026-09-10)

전 구간(full_workflow.py)을 돌리면 한 조에 45~60분이 걸려 변인통제를 못 한다.
여기서는 **봉투 + 파우더 + 잡아주는 것**만 남기고 10초를 흔든다 — 한 조 몇 분.

## 무엇을 묻나

full_workflow 의 파지→삽입 구간에서 봉투가 이산적으로 덜컥거린다. 지금까지 넷이
빗나갔다(전부 full_workflow 실측, `above` 구간 기준):

    dt 절반          z 가속도 부호반전 42.6% -> 40.0%   무변화
    substeps         IPC 경로에서 읽히지도 않음
    FEM_DAMPING 5배  42.6% -> 42.1%                     죽은 노브(커플러에 참조 없음)
    CLOTH_BEND 4배   42.6% -> 48.4%, 정점이동은 -17%    천은 원인이 아니라 전달경로

남은 유력 후보는 **구동 방식**이다. run_arm 은 매 스텝 `set_dofs_position` 으로
팔을 순간이동시키는데 속도는 갱신하지 않는다. 접촉 솔버에는 "움직이는 표면"이
아니라 "매 스텝 새 자리에서 밀어냄"으로 들어간다.

## 구동 방식 (DRIVE)

    spc      강체 없이 봉투 입구 정점의 SPC 목표를 해석적 사인파로 직접 흔든다.
             **매끄러운 경계의 기준선** — 여기서도 덜컥거리면 구동은 무죄다.
    set      강체 홀더를 set_pos 로만 옮긴다(속도 갱신 없음) = full_workflow 현행
    setvel   set_pos + set_dofs_velocity(유한차분) = 순간이동은 두고 속도만 일관되게

set/setvel 에서는 봉투 입구를 홀더의 **로컬 좌표**로 SPC 부착해 홀더의 실제 자세를
따라가게 한다(full_workflow 흡착과 같은 방식). 그래서 홀더가 덜컥이면 봉투도 덜컥인다.

## env

    DRIVE=spc|set|setvel   구동 방식 (기본 spc)
    DT_MS=5.0              타임스텝
    N_GRAINS=524           낟알 수 (0 이면 파우더 없음 = 대조군)
    GRAIN_RADIUS_MM=0.75   낟알 반지름
    GRAIN_RHO=2160         낟알 밀도 (소금)
    AMP_MM=40  FREQ_HZ=1.0  SECONDS=10   흔들기
    CLOTH_E=4.0e5  CLOTH_BEND=400  CLOTH_THICK_MM=1.0
    IPC_D_HAT=1e-4
    NO_VIDEO=1  TAG=...
"""
import os
import sys

import numpy as np
import trimesh as tm

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
sys.path.insert(0, os.path.join(_r, "Powder_flip_test"))
sys.path.insert(0, os.path.join(os.path.dirname(_r), "utills"))
import paths                                                       # noqa: E402
from fem_ipc_workarounds import patch_ipc_vertex_attach            # noqa: E402

from datetime import datetime                                      # noqa: E402

OUT = os.path.join(_r, "probe", "RESULT_shake")
os.makedirs(OUT, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")

DRIVE = os.environ.get("DRIVE", "spc").lower()
assert DRIVE in ("spc", "set", "setvel"), f"DRIVE={DRIVE!r}"
DT = float(os.environ.get("DT_MS", "5.0")) * 1e-3
N_GRAINS = int(os.environ.get("N_GRAINS", "524"))
GR = float(os.environ.get("GRAIN_RADIUS_MM", "0.75")) * 1e-3
GRHO = float(os.environ.get("GRAIN_RHO", "2160"))
AMP = float(os.environ.get("AMP_MM", "40")) * 1e-3
FREQ = float(os.environ.get("FREQ_HZ", "1.0"))
SECONDS = float(os.environ.get("SECONDS", "10"))
CLOTH_E = float(os.environ.get("CLOTH_E", "4.0e5"))
CLOTH_BEND = float(os.environ.get("CLOTH_BEND", "400"))
CLOTH_THICK = float(os.environ.get("CLOTH_THICK_MM", "1.0")) * 1e-3
CLOTH_NU, CLOTH_RHO, CLOTH_FRICTION = 0.499, 200.0, 0.8
D_HAT = float(os.environ.get("IPC_D_HAT", "1e-4"))
NO_VIDEO = os.environ.get("NO_VIDEO", "0") == "1"
TAG = os.environ.get("TAG", DRIVE)

BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag",
                       os.environ.get("BAG_STL_NAME",
                                      "Samplebag_seal_pouch3_sealslab3mm.stl"))
BAG_EULER = (90, 0, 90)
BAG_POS = (0.0, 0.0, 0.25)
HOLDER = (0.030, 0.030, 0.010)      # 홀더 박스 크기 [m]


def _npy(x):
    x = x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)
    return x[0] if x.ndim == 3 else x


def main():
    import genesis as gs
    patch_ipc_vertex_attach(strength_rate=1e4)
    from ipc_grain_coupler import _build_grain_coupler_class
    import genesis.utils.geom as gu
    import uipc

    gs.init(backend=gs.gpu, logging_level="warning", precision="32")
    n_steps = int(round(SECONDS / DT))

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=D_HAT, contact_friction_enable=True, two_way_coupling=True,
            enable_rigid_rigid_contact=False, enable_rigid_ground_contact=False,
            constraint_strength_translation=100.0, constraint_strength_rotation=100.0,
        ),
        vis_options=gs.options.VisOptions(
            background_color=(0.93, 0.94, 0.96), ambient_light=(0.16, 0.16, 0.18),
            lights=[{"type": "directional", "dir": (-1, -1, -1),
                     "color": (1.0, 1.0, 1.0), "intensity": 6.0}]),
        show_viewer=False,
    )
    GrainCoupler = _build_grain_coupler_class()
    coupler = GrainCoupler(scene._sim, scene._sim.coupler_options)
    scene._sim._coupler = coupler

    bag = scene.add_entity(
        material=gs.materials.FEM.Cloth(E=CLOTH_E, nu=CLOTH_NU, rho=CLOTH_RHO,
                                        thickness=CLOTH_THICK,
                                        bending_stiffness=CLOTH_BEND,
                                        friction_mu=CLOTH_FRICTION),
        morph=gs.morphs.Mesh(file=BAG_STL, scale=1.0, pos=BAG_POS, euler=BAG_EULER),
        surface=gs.surfaces.Default(color=(0.75, 0.78, 0.85, 0.55), double_sided=True),
    )

    holder = None
    if DRIVE in ("set", "setvel"):
        # 봉투 입구 위에 놓는 판. 봉투는 이 판의 **로컬 좌표**로 SPC 부착되므로
        # 판의 실제 자세를 그대로 따라간다 — 판이 덜컥이면 봉투도 덜컥인다.
        holder = scene.add_entity(
            material=gs.materials.Rigid(coup_type="two_way_soft_constraint",
                                        coup_friction=CLOTH_FRICTION),
            surface=gs.surfaces.Default(color=(0.85, 0.35, 0.25)),
            # 봉투 상단(z = BAG_POS[2] + 45mm)보다 확실히 위에 둔다. 맞닿으면
            # libuipc 가 초기 겹침으로 world 를 무효화한다("World is not valid"
            # -> body_count AttributeError, 2026-09-10 실측). 봉투는 접촉이
            # 아니라 SPC 로 홀더에 매달리므로 떨어져 있어도 된다.
            morph=gs.morphs.Box(pos=(BAG_POS[0], BAG_POS[1],
                                     BAG_POS[2] + 0.045 + HOLDER[2] / 2 + 0.010),
                                size=HOLDER, fixed=False))

    # 낟알을 봉투 안에 직접 생성 (bbox 로는 못 잡는다 — 파우치는 상자가 아니다)
    grain_spec = None
    if N_GRAINS > 0:
        bm = tm.load(BAG_STL).copy()
        bm.vertices = ((gu.quat_to_R(gu.xyz_to_quat(np.array(BAG_EULER), rpy=True,
                                                    degrees=True)) @ bm.vertices.T).T
                       + np.array(BAG_POS))
        pq = tm.proximity.ProximityQuery(bm)
        mg = CLOTH_THICK + GR + D_HAT + 0.0002
        rng = np.random.default_rng(0)
        cand = rng.uniform(*bm.bounds, size=(max(60000, N_GRAINS * 60), 3))
        cand = cand[np.abs(pq.signed_distance(cand)) >= mg]
        dmin = 2 * GR + 4 * D_HAT
        pts = np.empty((0, 3))
        for q in cand:
            if len(pts) == 0 or np.linalg.norm(pts - q, axis=1).min() >= dmin:
                pts = np.vstack([pts, q])
            if len(pts) >= N_GRAINS:
                break
        m1 = GRHO * (4.0 / 3.0) * np.pi * GR ** 3
        grain_spec = coupler.add_grains(pts, radius=GR, mass_density=GRHO,
                                        friction_mu=0.6)
        print(f"[shake] 낟알 {len(pts)}/{N_GRAINS}알  R={GR*1e3:.2f}mm  "
              f"총 {m1*len(pts)*1e3:.3f}g  후보 {len(cand)}점  여유 {mg*1e3:.2f}mm")
    else:
        print("[shake] 파우더 없음 (대조군)")

    cam = scene.add_camera(res=(960, 720), pos=(0.42, -0.38, 0.30),
                           lookat=(0.0, 0.0, 0.21), fov=42, GUI=False, debug=True)

    print(f"[shake] DRIVE={DRIVE} dt={DT*1e3:.2f}ms  {n_steps}스텝  "
          f"진폭 {AMP*1e3:.0f}mm @ {FREQ:.2f}Hz  {SECONDS:.0f}s")
    scene.build(n_envs=0)

    # 봉투 입구(상단) 정점을 잡는다
    vp0 = _npy(bag.get_state().pos).squeeze()
    mouth = np.where(vp0[:, 2] >= np.quantile(vp0[:, 2], 0.93))[0]
    slot = scene.sim.coupler.cloth_slots[(bag, 0)]

    def _spc(idx, tgt):
        g = slot.geometry()
        ic = uipc.view(g.vertices().find(uipc.builtin.is_constrained))
        ap = uipc.view(g.vertices().find(uipc.builtin.aim_position))
        ic[:] = 0
        ic[idx] = 1
        ap[idx] = np.asarray(tgt, dtype=np.float64).reshape(-1, 3, 1)

    p_mouth0 = vp0[mouth].copy()
    loc = hz0 = None
    if holder is not None:
        h0 = _npy(holder.get_pos()).reshape(-1)[:3]
        loc = p_mouth0 - h0            # 홀더 로컬 좌표로 저장
        hz0 = float(h0[2])
    _spc(mouth, p_mouth0)
    print(f"[shake] 입구 정점 {len(mouth)}개 부착"
          + ("" if holder is None else f", 홀더 z={hz0*1e3:.1f}mm"))

    mp4 = os.path.join(OUT, f"shake_{TAG}_{_TS}.mp4")
    if not NO_VIDEO:
        cam.start_recording(save_to_filename=mp4, fps=30)

    rec = []
    prev_vp = None
    prev_cmd = np.array([BAG_POS[0], BAG_POS[1], 0.0])
    for k in range(n_steps):
        t = (k + 1) * DT
        dx = AMP * np.sin(2 * np.pi * FREQ * t)
        if DRIVE == "spc":
            tgt = p_mouth0.copy()
            tgt[:, 0] += dx
            _spc(mouth, tgt)
        else:
            cmd = np.array([BAG_POS[0] + dx, BAG_POS[1], hz0])
            holder.set_pos(cmd)
            if DRIVE == "setvel":
                v = (cmd - prev_cmd) / DT if k else np.zeros(3)
                holder.set_dofs_velocity(np.concatenate([v, np.zeros(3)]))
            prev_cmd = cmd
            hp = _npy(holder.get_pos()).reshape(-1)[:3]
            _spc(mouth, loc + hp)      # 홀더 실제 위치를 따라간다

        scene.step()
        if not NO_VIDEO:
            if grain_spec is not None:
                scene.clear_debug_objects()
                scene.draw_debug_spheres(coupler.get_grain_positions(grain_spec),
                                         radius=GR, color=(0.85, 0.75, 0.55, 1.0))
            cam.render()

        vp = _npy(bag.get_state().pos).squeeze()
        com = vp.mean(axis=0)
        dmax = 0.0 if prev_vp is None else float(np.linalg.norm(vp - prev_vp, axis=1).max())
        prev_vp = vp.copy()
        hx = (np.nan if holder is None
              else float(_npy(holder.get_pos()).reshape(-1)[0]))
        rec.append((t, dx, hx, com[0], com[1], com[2], dmax,
                    float(vp[:, 0].max() - vp[:, 0].min()),
                    float(vp[:, 2].max() - vp[:, 2].min())))
        if (k + 1) % 200 == 0:
            print(f"[shake] {k+1:5d}/{n_steps}  cmd_x={dx*1e3:+7.2f}mm  "
                  f"com_x={com[0]*1e3:+7.2f}  폭={rec[-1][7]*1e3:5.1f}  "
                  f"높이={rec[-1][8]*1e3:5.1f}mm", flush=True)

    if not NO_VIDEO:
        cam.stop_recording()
        print(f"[saved] {mp4}")

    A = np.asarray(rec)
    t, dx, hx, com, dmax = A[:, 0], A[:, 1], A[:, 2], A[:, 3:6], A[:, 6]
    d = np.linalg.norm(np.diff(com, axis=0), axis=1) * 1e3
    acc = np.diff(np.diff(com[:, 0]))
    fl = int((np.sign(acc[1:]) * np.sign(acc[:-1]) < 0).sum())

    # ── 덜컥거림을 **크기**로 잰다 ────────────────────────────────────────
    # 부호반전율만으로는 못 가른다. 2차 차분은 아무리 작은 잡음에도 부호가 매
    # 스텝 뒤집혀 50% 근처로 포화된다(실측: 매끄러운 사인파 구동인 DRIVE=spc
    # 에서도 49.2%). 그래서 두 가지를 같이 본다:
    #
    #   jerk_ratio  실측 |2차차분| 중앙값 / 지령 사인파의 이론 |2차차분|.
    #               1 이면 지령만큼만 움직인 것, 크면 그 배수만큼 잡음이 얹힌 것.
    #   jitter_rms  이동평균(1/10 주기)을 뺀 잔차의 RMS [mm] = 눈에 보이는 떨림 폭.
    _acc_cmd = AMP * (2 * np.pi * FREQ) ** 2 * DT ** 2          # [m], 매끄러운 경우
    jerk_ratio = float(np.median(np.abs(acc)) / max(_acc_cmd, 1e-15))
    _w = max(3, int(round(1.0 / max(FREQ, 1e-6) / DT / 10)))
    _ker = np.ones(_w) / _w
    _sm = np.convolve(com[:, 0], _ker, mode="same")
    _res = (com[:, 0] - _sm)[_w:-_w] if len(com) > 2 * _w else (com[:, 0] - _sm)
    jitter_rms = float(np.sqrt(np.mean(_res ** 2)) * 1e3)

    npz = os.path.join(OUT, f"shake_{TAG}_{_TS}.npz")
    np.savez(npz, t=t, cmd_x=dx, holder_x=hx, com=com, dmax=dmax,
             width=A[:, 7], height=A[:, 8], dt=DT, drive=DRIVE, n_grains=N_GRAINS,
             jerk_ratio=jerk_ratio, jitter_rms=jitter_rms, flip_pct=100 * fl / max(1, len(acc) - 1))
    print()
    print("=" * 68)
    print(f"[RESULT] DRIVE={DRIVE} dt={DT*1e3:.2f}ms 낟알={N_GRAINS} "
          f"E={CLOTH_E:.1e} bend={CLOTH_BEND:.0f}")
    print(f"[RESULT] 스텝당 COM 이동(mm) 50/90/99/max: "
          f"{np.round(np.quantile(d, [.5, .9, .99, 1.0]), 3)}")
    print(f"[RESULT] 스텝당 정점최대이동(mm) 50/90/99/max: "
          f"{np.round(np.quantile(dmax[1:] * 1e3, [.5, .9, .99, 1.0]), 3)}")
    print(f"[RESULT] x 가속도 부호반전 {100*fl/max(1,len(acc)-1):.1f}% "
          f"({fl}/{len(acc)-1}) — 작은 잡음에도 50%로 포화되니 크기와 같이 볼 것")
    print(f"[RESULT] **jerk_ratio {jerk_ratio:8.1f}** "
          f"(1 이면 지령만큼만 움직임, 클수록 잡음이 얹힘)")
    print(f"[RESULT] **jitter_rms {jitter_rms:8.4f} mm** (이동평균 뺀 잔차 RMS)")
    if holder is not None:
        lag = np.abs(hx - (BAG_POS[0] + dx)) * 1e3
        print(f"[RESULT] 홀더 추종오차(mm) 평균 {lag.mean():.3f} 최대 {lag.max():.3f}")
    print(f"[RESULT] 봉투 폭 {A[:, 7].min()*1e3:.1f}~{A[:, 7].max()*1e3:.1f}mm  "
          f"높이 {A[:, 8].min()*1e3:.1f}~{A[:, 8].max()*1e3:.1f}mm")
    print(f"[saved] {npz}")
    print("=" * 68)


if __name__ == "__main__":
    main()

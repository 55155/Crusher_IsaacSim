"""probe_contact_force.py — 입자별 **접촉력**을 uipc 에서 꺼낼 수 있는가 (2026-09-11)

물음: Genesis IPC 커플러는 낟알 상태로 **좌표만** 회수한다
(`_retrieve_grain_states`). 그래서 지금까지 알당 반력을 좌표 2계 차분(=알짜힘)
으로만 냈는데, 그건 준정적 압착에서 0 에 수렴해 **접촉 하중을 못 잰다**
(docs/DigitalTwin.md §27-5). uipc 가 내부에서 계산하는 접촉력을 직접 꺼낼 수
있으면 그 한계가 사라진다.

후보: `uipc.core.ContactSystemFeature` — "Feature for computing contact energy,
gradients, and Hessians". `contact_gradient` 는 ∂E_contact/∂x 이므로
**접촉력 = −gradient** 다.

검증 설계(최소 씬): 바닥 평면 + 낟알을 쌓아 정착시킨 뒤
  1) feature 가 실제로 잡히는가
  2) `contact_gradient` 가 정점별 값을 채워 주는가
  3) **물리 검산** — 정착한 더미에서 접촉력 합 ≈ 총 무게여야 한다
     (정적 평형: 바닥이 받치는 힘 = mg). 이게 맞으면 단위·부호까지 확정된다.

**결론(2026-09-11 실측): 꺼낼 수 있다.**

    world.features().find(uipc.core.ContactSystemFeature)
    csf.contact_gradient(prim_type, geom)      # geom.instances() 에 채워짐
        grad : (n,3,1)  정점별 ∂E/∂x
        i    : (n,)     전역 정점 인덱스
    접촉력 = -grad / dt^2

`prim_type` 은 10종(`PT/PP/EE/PE/PH` x `N`(법선)/`F`(마찰)). 낟알(점군)에서는
낟알-낟알이 `PP`, 낟알-바닥평면이 `PH` 다. 빈 타입은 size 0 으로 돌아온다.

**dt^2 스케일이 핵심이다.** IPC 는 증분 포텐셜
`E = 0.5|x-x~|^2_M + dt^2*(E_elastic + E_contact)` 를 최소화하므로 gradient 에
dt^2 이 이미 곱해져 있다. 실측: 120알을 4,000스텝 정착시킨 뒤
`무게/원시z합 = 40000.0` 이고 이는 `1/dt^2`(dt=5e-3) 와 정확히 일치한다.
보정 후 `z합/무게 = 1.0000`, 정점별 |f| 가 전부 1알 자중(0.0616mN)이다.

정착이 덜 되면 이 비가 안 맞는다(600스텝에서 6640). 검산은 반드시 속도가
충분히 0 에 간 뒤에 해야 한다 — 스크립트가 정착 중 속도를 같이 찍는다.
"""
import os, sys
import numpy as np

_r = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_r, "Powder_flip_test"))

DT = 5e-3
N_GRAINS = int(os.environ.get("N_GRAINS", "120"))
R = float(os.environ.get("GRAIN_RADIUS_MM", "1.0")) * 1e-3
RHO = float(os.environ.get("GRAIN_RHO", "1500"))
D_HAT = float(os.environ.get("D_HAT", "1e-4"))
N_SETTLE = int(os.environ.get("N_SETTLE", "600"))


def main():
    import genesis as gs
    from ipc_grain_coupler import _build_grain_coupler_class
    import uipc
    import uipc.core as ucore
    import uipc.geometry as ugeo

    gs.init(backend=gs.gpu, logging_level="warning", precision="32", seed=0)
    GrainIPCCoupler = _build_grain_coupler_class()
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=D_HAT, contact_friction_enable=True, two_way_coupling=True,
            enable_rigid_rigid_contact=False, enable_rigid_ground_contact=True),
        show_viewer=False)
    coupler = GrainIPCCoupler(scene._sim, scene._sim.coupler_options)
    scene._sim._coupler = coupler
    scene.add_entity(gs.morphs.Plane(pos=(0, 0, 0)),
                     material=gs.materials.Rigid(coup_type="ipc_only"))

    # 좁은 기둥에 쌓아 바닥에서 서로 눌리게 한다 — 접촉력이 0 이 아니어야 의미가 있다.
    rng = np.random.default_rng(0)
    sp = 2 * R + 4 * D_HAT
    n_side = int(np.ceil(np.sqrt(N_GRAINS / 6.0)))
    pts = []
    for i in range(N_GRAINS):
        gx, gy = i % n_side, (i // n_side) % n_side
        gz = i // (n_side * n_side)
        pts.append((gx * sp * 1.05 - n_side * sp / 2,
                    gy * sp * 1.05 - n_side * sp / 2,
                    R + 0.002 + gz * sp * 1.05))
    pts = np.array(pts) + rng.uniform(-R * 0.05, R * 0.05, size=(N_GRAINS, 3))
    spec = coupler.add_grains(pts, radius=R, mass_density=RHO, friction_mu=0.6)
    scene.build(n_envs=0)

    m1 = RHO * (4.0 / 3.0) * np.pi * R ** 3
    W = m1 * 9.81 * N_GRAINS
    print(f"[probe] 낟알 {N_GRAINS}알 R={R*1e3:.2f}mm  1알 {m1*1e6:.3f}mg  총무게 {W*1e3:.4f} mN")

    for k in range(N_SETTLE):
        scene.step()
        if (k + 1) % 500 == 0:
            _a = coupler.get_grain_positions(spec).copy()
            scene.step()
            _b = coupler.get_grain_positions(spec)
            _v = np.linalg.norm((_b - _a) / DT, axis=1)
            print(f"   [settle {k+1:5d}] 속도 평균 {_v.mean()*1e6:8.2f} um/s  "
                  f"최대 {_v.max()*1e6:9.2f} um/s")
    p0 = coupler.get_grain_positions(spec).copy()
    scene.step()
    p = coupler.get_grain_positions(spec)
    vmag = np.linalg.norm((p - p0) / DT, axis=1)
    print(f"[probe] 측정 시점 속도: 평균 {vmag.mean()*1e6:.3f} um/s  최대 {vmag.max()*1e6:.3f} um/s "
          f"(정지 판정용 — 클수록 정적평형 검산이 흐려진다)")
    print(f"[probe] 정착 후 z {p[:,2].min()*1e3:.2f} ~ {p[:,2].max()*1e3:.2f} mm  "
          f"평균속도 확인용 재스텝 전")

    # ── 1) feature 조회 ────────────────────────────────────────────────
    world = coupler._ipc_world
    feats = world.features()
    print(f"[probe] world.features() = {type(feats).__name__}")
    try:
        print("[probe] features json:", feats.to_json())
    except Exception as e:
        print("[probe] to_json 실패:", repr(e))
    csf = None
    for key in (ucore.ContactSystemFeature, "ContactSystemFeature",
                getattr(ucore.ContactSystemFeature, "FeatureName", None)):
        if key is None:
            continue
        try:
            got = feats.find(key)
            if got is not None:
                csf = got
                print(f"[probe] **ContactSystemFeature 확보** (find 인자={key!r}) "
                      f"name={got.name()} type={got.type_name()}")
                break
        except Exception as e:
            print(f"[probe] find({key!r}) 실패: {e!r}")
    if csf is None:
        print("[RESULT] ContactSystemFeature 를 못 얻었다 — 이 백엔드는 미지원")
        return

    print("[probe] contact_primitive_types:", csf.contact_primitive_types())

    # ── 2) contact_gradient 호출 ──────────────────────────────────────
    tot = np.zeros(3)
    per_vert = {}
    for pt in csf.contact_primitive_types():
        g = ugeo.Geometry()
        try:
            csf.contact_gradient(pt, g)
        except Exception as e:
            print(f"[probe] contact_gradient({pt!r}) 실패: {type(e).__name__}: {e}")
            continue
        inst = g.instances()
        n = inst.size()
        if n == 0:
            print(f"[probe] prim={pt!r:6s} 접촉 없음")
            continue
        grad = np.asarray(inst.find("grad").view()).reshape(n, 3)
        vidx = np.asarray(inst.find("i").view()).reshape(n)
        f = -grad                       # 접촉력 = -∂E/∂x
        tot += f.sum(axis=0)
        for vi, fv in zip(vidx, f):
            per_vert[int(vi)] = per_vert.get(int(vi), np.zeros(3)) + fv
        nn = np.linalg.norm(f, axis=1)
        print(f"[probe] prim={pt!r:6s} 항목 {n:5d}  |f|합 {nn.sum()*1e3:9.4f} mN  "
              f"z합 {f[:,2].sum()*1e3:+9.4f} mN  최대 {nn.max()*1e3:.4f} mN  "
              f"정점 {len(set(vidx.tolist()))}개")

    print()
    print(f"[probe] 원시 z합 = {tot[2]:.6e} N(스케일 미보정)   총무게 {W:.6e} N")
    print(f"[probe] 무게/원시z합 = {W/tot[2]:.1f}   1/dt^2 = {1/DT**2:.1f}   "
          f"1/dt = {1/DT:.1f}")
    # IPC 는 증분 포텐셜 E = 0.5|x-x~|^2_M + dt^2*(E_elastic+E_contact) 를 푼다.
    # 따라서 contact_gradient 는 dt^2 이 곱해진 값이고, 물리 힘은 /dt^2 이다.
    SC = 1.0 / DT**2
    tot = tot * SC
    per_vert = {k: v * SC for k, v in per_vert.items()}
    print(f"[RESULT] 전 타입 합력(/dt^2) = ({tot[0]*1e3:+.4f}, {tot[1]*1e3:+.4f}, "
          f"{tot[2]*1e3:+.4f}) mN")
    print(f"[RESULT] 총 무게            = {W*1e3:.4f} mN   ->  z합/무게 = {tot[2]/W:+.4f}")
    print(f"[RESULT] 접촉력이 붙은 정점 {len(per_vert)}개 / 낟알 {N_GRAINS}알")
    if per_vert:
        mags = np.array([np.linalg.norm(v) for v in per_vert.values()])
        q = np.quantile(mags, [0.5, 0.9, 1.0]) * 1e3
        print(f"[RESULT] 정점별 |f| 중앙 {q[0]:.4f} / 90% {q[1]:.4f} / 최대 {q[2]:.4f} mN "
              f"(1알 자중 {m1*9.81*1e3:.4f} mN)")
        print(f"[RESULT] 자중 대비 최대 {mags.max()/(m1*9.81):.1f}배")
    ok = abs(tot[2] / W - 1.0) < 0.02
    print()
    print("=" * 70)
    print(f"[VERDICT] {'성립' if ok else '불일치'} — 접촉력 = -contact_gradient / dt^2")
    print(f"          정적평형 검산 z합/무게 = {tot[2]/W:.4f} (1.0 이어야 함)")
    if not ok:
        print("          더미가 아직 안 멈췄을 수 있다 — N_SETTLE 을 키워 재확인할 것")
    print("=" * 70)


if __name__ == "__main__":
    main()

"""probe_grain_contact_law.py — 낟알 접촉법칙 F(간격) 을 직접 잰다 (2026-09-29)

사용자 질문: "PINN 에 절댓값으로 써도 되나, 아니면 상대값 정도는 되나. 검증하려면
FEM.grain(점질량)과 FEM.Elastic 을 비교하면 될 것 같다."

방향은 맞다. 다만 비교 전에 **척도 계산**이 실험 설계를 바꾼다:

    소금 알갱이 E≈40GPa, nu=0.25, R=1mm, F=443mN(측정된 알별 하중)
    Hertz  delta = (3F / (4 E* sqrt(R/2)))^(2/3) = 0.79 um
    그런데 IPC_D_HAT = 100 um

**배리어 간격이 물리적 접촉 변형의 127배다.** 그러면 탄성체로 바꿔도 알의 변형이
배리어 standoff 옆에서 무시되므로 점질량과 거의 같은 힘이 나온다 — 즉 판별 변수는
구성모델이 아니라 `d_hat` 이라는 예측이 선다. 이 프로브가 그 예측을 재는 것이다.

측정 설계 — **두 알 하나짜리 접촉**이 최소이고, 해석 기준(Hertz)이 존재한다:

  · 알 A, B 를 SPC 로 붙잡고 B 의 목표를 단계적으로 A 쪽으로 밀어넣는다.
  · SPC 가 목표에 정확히 도달하지 못해도 **상관없다** — 매 단계에서 실제 중심거리와
    실제 접촉력을 둘 다 재기 때문이다. (d, F) 쌍만 모이면 법칙이 나온다.
  · 접촉력은 §27-13 의 쌍 복원으로 읽는다: PP 두 행 중 한쪽 = -grad/dt^2.
  · 수렴이 전제다(§27-12) — NEWTON_TOL 을 조이고 단계마다 충분히 정착시킨다.

모드:
  GRAIN_MODEL=particle  uipc Particle (현행). thickness=R 이 곧 접촉 반경.
  GRAIN_MODEL=elastic   FEM.Elastic 구 두 개(tet). 표면이 실제 메시다.

출력: (간격, 힘) 표 + Hertz 해석값과의 비. 간격 정의를 두 모드에서 같게 두려고
**중심거리 - 2R** 로 통일한다(Particle 은 정의상, Elastic 은 구 반경 R 이므로).
"""
import os, sys, json
import numpy as np

_r = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_r, "Powder_flip_test"))
sys.path.insert(0, os.path.join(os.path.dirname(_r), "utills"))

DT = float(os.environ.get("DT_MS", "5.0")) * 1e-3
R = float(os.environ.get("GRAIN_RADIUS_MM", "1.0")) * 1e-3
RHO = float(os.environ.get("GRAIN_RHO", "2160"))          # NaCl
D_HAT = float(os.environ.get("D_HAT", "1e-4"))
NEWTON_TOL = float(os.environ.get("NEWTON_TOL", "1e-4"))
MODEL = os.environ.get("GRAIN_MODEL", "particle").lower()
SPC_K = float(os.environ.get("SPC_K", "1e6"))
N_SETTLE = int(os.environ.get("N_SETTLE", "400"))         # 단계당 정착 스텝
# 목표 **간격**(중심거리 - 2R) [um] — 배리어 d_hat(100um)를 걸치도록 잡는다:
# 바깥(힘 0) -> 배리어 안 -> 접촉면 침입.
GAP_UM = [float(x) for x in os.environ.get(
    "GAP_UM", "300,200,150,120,100,80,60,40,20,10,5,0,-5,-10").split(",")]
# **스폰은 반드시 떨어뜨린다.** 중심거리 2R 로 두면 libuipc sanity check 가
# "too close (distance=thickness)" 로 world 를 무효화한다(실측 2026-09-29).
SPAWN_GAP = float(os.environ.get("SPAWN_GAP_UM", "500")) * 1e-6
# 탄성 물성 — 기본은 소금. 너무 딱딱하면 Newton 이 고생하므로 스윕 가능하게 뺀다.
E_PA = float(os.environ.get("E_PA", "4.0e10"))
NU = float(os.environ.get("NU", "0.25"))
ELASTIC_SUBDIV = int(os.environ.get("ELASTIC_SUBDIV", "2"))
_ELASTIC_REST = {}          # 엔티티 -> 초기 정점 좌표(구속 목표 계산용)


def _npy(x):
    """CUDA 텐서를 numpy 로 — Genesis 상태는 torch cuda 텐서다."""
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def hertz_force(delta, E=E_PA, nu=NU, r=R):
    """동일 구 두 개의 Hertz 접촉력 [N]. delta = 상호 접근량 [m]."""
    Es = E / (2.0 * (1.0 - nu ** 2))       # 1/E* = 2(1-nu^2)/E
    Reff = r / 2.0
    d = np.maximum(np.asarray(delta, float), 0.0)
    return (4.0 / 3.0) * Es * np.sqrt(Reff) * d ** 1.5


def sphere_tets(r_mm, subdiv=1):
    """구의 tet 메시 — icosphere 표면 + 중심으로 부채꼴(star-shaped 이라 유효).

    make_capsule_tets_v2 의 medial-axis 부채꼴과 같은 방식이다. 구는 중심 하나로
    충분하고 sliver 가 안 생긴다(표면 삼각형이 정삼각형에 가까우므로).
    """
    import trimesh as tm
    ico = tm.creation.icosphere(subdivisions=subdiv, radius=r_mm)
    v = np.asarray(ico.vertices, float)
    f = np.asarray(ico.faces, np.int64)
    ci = len(v)
    verts = np.vstack([v, [[0.0, 0.0, 0.0]]])
    # tet = (표면삼각형 3점, 중심). 부호(양의 부피)를 맞춘다.
    tets = np.column_stack([f, np.full(len(f), ci)])
    a, b, c, d = (verts[tets[:, 0]], verts[tets[:, 1]],
                  verts[tets[:, 2]], verts[tets[:, 3]])
    vol = np.einsum("ij,ij->i", np.cross(b - a, c - a), d - a) / 6.0
    flip = vol < 0
    tets[flip, 1], tets[flip, 2] = tets[flip, 2], tets[flip, 1].copy()
    return verts, tets


def main():
    import genesis as gs
    from ipc_grain_coupler import _build_grain_coupler_class
    import uipc as _u
    import uipc.core as ucore
    import uipc.geometry as ugeo

    gs.init(backend=gs.gpu, logging_level="warning", precision="64", seed=0)
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, 0)),   # 중력 OFF
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=D_HAT, contact_friction_enable=False, two_way_coupling=True,
            enable_rigid_rigid_contact=False, enable_rigid_ground_contact=False,
            newton_tolerance=NEWTON_TOL),
        show_viewer=False)
    coupler = _build_grain_coupler_class()(scene._sim, scene._sim.coupler_options)
    scene._sim._coupler = coupler

    # 중력을 끈 이유: 자중(0.0887 mN)이 재려는 접촉력(수백 mN)보다 훨씬 작지만,
    # 두 알을 SPC 로 세우는 실험에서 중력은 축을 흔드는 교란일 뿐 정보가 없다.
    m1 = RHO * (4.0 / 3.0) * np.pi * R ** 3
    print(f"[probe] MODEL={MODEL}  R={R*1e3:.2f}mm  rho={RHO:.0f}  "
          f"1알 {m1*1e6:.4f}mg  d_hat={D_HAT*1e6:.1f}um  newton_tol={NEWTON_TOL:g}")
    if MODEL == "elastic":
        print(f"[probe] E={E_PA:.3g} Pa  nu={NU}  subdiv={ELASTIC_SUBDIV}")

    _h0 = R + SPAWN_GAP / 2.0                            # 중심거리 = 2R + SPAWN_GAP
    x0 = np.array([[-_h0, 0.0, 0.0], [_h0, 0.0, 0.0]])
    ents = []
    if MODEL == "particle":
        spec = coupler.add_grains(x0, radius=R, mass_density=RHO, friction_mu=0.0,
                                  spc_strength=SPC_K)
    else:
        from primitive_tablet_generator import add_analytic_fem_entity
        # IPC 커플러에서 FEM 정점 구속이 예외를 던지는 버그 우회(full_workflow 와 동일)
        from fem_ipc_workarounds import patch_fem_vertex_constraints
        patch_fem_vertex_constraints()
        v_mm, tets = sphere_tets(R * 1e3, ELASTIC_SUBDIV)
        print(f"[probe] 구 tet: 정점 {len(v_mm)}  tet {len(tets)}")
        mat = gs.materials.FEM.Elastic(E=E_PA, nu=NU, rho=RHO,
                                       model="stable_neohookean")
        for i, c in enumerate(x0):
            _e = add_analytic_fem_entity(
                scene, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    f"_sphere_{i}.stl"),
                v_mm, tets, mat, scale=1e-3, pos=tuple(c))
            ents.append(_e)
            _ELASTIC_REST[id(_e)] = v_mm * 1e-3 + np.asarray(c)
        spec = None
    scene.build(n_envs=0)

    csf = coupler._ipc_world.features().find(ucore.ContactSystemFeature)
    ptypes = list(csf.contact_primitive_types())

    def centers():
        if MODEL == "particle":
            return coupler.get_grain_positions(spec)
        return np.array([_npy(e.get_state().pos).squeeze().mean(axis=0)
                         for e in ents])

    def surface_gap():
        """두 물체 **표면** 최근접 거리 [m]. 두 모드를 같은 자로 재기 위한 것.

        Particle 은 표면이 점에서 thickness=R 이므로 중심거리-2R 이 곧 표면 간격이다.
        Elastic 은 정점이 실제 표면이라 정점쌍 최소거리를 쓴다 — 중심거리-2R 로
        재면 icosphere 의 면이 반지름 안쪽으로 들어가 있어(subdiv 1 에서 수십 um)
        간격을 과소평가한다. 실측에서 이 차이가 접촉 항목 0 의 원인이었다.
        """
        if MODEL == "particle":
            c = centers()
            return float(np.linalg.norm(c[1] - c[0])) - 2 * R
        from scipy.spatial import cKDTree
        a = _npy(ents[0].get_state().pos).squeeze()
        b = _npy(ents[1].get_state().pos).squeeze()
        return float(cKDTree(a).query(b)[0].min())

    def contact_force():
        """두 물체 사이 접촉 합력 [N] — 한쪽이 받는 힘. 법선만(마찰 OFF)."""
        tot = np.zeros(3)
        n_items = 0
        for pt in ptypes:
            if pt.endswith("+F"):
                continue
            G = ugeo.Geometry()
            csf.contact_gradient(pt, G)
            inst = G.instances()
            n = inst.size()
            if n == 0:
                continue
            gr = np.asarray(inst.find("grad").view()).reshape(n, 3)
            ix = np.asarray(inst.find("i").view()).ravel()
            f = -gr / (DT * DT)
            # 전역 정점 인덱스가 작은 절반(물체 A)이 받는 힘만 모은다.
            half = ix < np.median(ix) + 0.5 if n > 1 else np.ones(n, bool)
            tot += f[half].sum(axis=0)
            n_items += n
        return tot, n_items

    print(f"\n{'목표간격':>9s} {'실제간격':>10s} {'접촉력':>12s} {'Hertz(해석)':>13s} "
          f"{'비 sim/Hertz':>12s} {'접촉항목':>8s}")
    rows = []
    for gap_um in GAP_UM:
        gap_t = 2 * R + gap_um * 1e-6                     # 목표 중심거리
        aim = np.array([[-gap_t / 2, 0.0, 0.0], [gap_t / 2, 0.0, 0.0]])
        if MODEL == "particle":
            coupler.hold_grains(spec, targets=aim)
        else:
            # **전 정점을 구속하면 안 된다.** hard constraint 는 매 스텝 위치를
            # 덮어써 접촉을 무시하므로(실측: -10um 관통에 접촉 항목 0), 구가
            # 변형될 자리가 없어 탄성을 아예 시험하지 못한다. 접촉면 반대쪽
            # **바깥 반구만** 잡고 접촉 쪽은 자유로 둔다 = 표준 압축시험.
            for e, c, a, sgn in zip(ents, x0, aim, (-1.0, +1.0)):
                vp = _npy(e.get_state().pos).squeeze()
                v0 = _ELASTIC_REST[id(e)]
                held = np.flatnonzero(sgn * (v0[:, 0] - v0[:, 0].mean()) > 0.5 * R)
                e.set_vertex_constraints(
                    verts_idx_local=held,
                    target_poss=v0[held] - v0.mean(axis=0) + a)
        for _ in range(N_SETTLE):
            scene.step()
        c = centers()
        d_center = float(np.linalg.norm(c[1] - c[0]))
        gap = surface_gap()                               # +면 떨어짐, -면 침입
        F, ni = contact_force()
        Fm = float(np.linalg.norm(F))
        # Hertz 는 침입량만 정의된다 — 떨어져 있으면 0.
        Fh = float(hertz_force(max(-gap, 0.0)))
        rows.append((gap_um, gap, Fm, Fh, ni))
        print(f"{gap_um:+7.1f}um {gap*1e6:+8.2f}um {Fm*1e3:10.4f}mN "
              f"{Fh*1e3:11.4f}mN {(Fm/Fh if Fh > 0 else np.nan):12.3f} {ni:8d}")

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       f"_contact_law_{MODEL}_dhat{D_HAT*1e6:.0f}um.json")
    with open(out, "w", encoding="utf-8") as fp:
        json.dump(dict(model=MODEL, R=R, rho=RHO, d_hat=D_HAT, E=E_PA, nu=NU,
                       newton_tol=NEWTON_TOL, dt=DT,
                       rows=[dict(gap_target_um=a, gap_m=b, F_N=c, F_hertz_N=d, n_items=e)
                             for a, b, c, d, e in rows]), fp, indent=1)
    print(f"[saved] {out}")


if __name__ == "__main__":
    main()

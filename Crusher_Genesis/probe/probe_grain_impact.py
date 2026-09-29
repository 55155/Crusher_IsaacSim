"""probe_grain_impact.py — 점질량 낟알 vs 탄성체 낟알, **충격 등가성** (2026-09-29)

사용자 질문: "지금 배선은 elastic 이 계산비용이 커서, 어차피 노말 방향으로 파쇄하니
점질량으로 나타내도 운동량이나 에너지에 큰 차이가 없다고 가정했다. 이게 물리적으로
충실한가? FEM.elastic 과의 비교가 필요하다."

`probe_grain_contact_law.py` 로 준정적 압축을 재려 했으나 **IPC 커플러 아래서는
Genesis FEM 정점 구속으로 구동이 안 된다**는 것이 실측으로 확인됐다:

    전 정점 hard constraint  -> 위치를 덮어써 접촉을 무시(-10um 관통, 접촉 항목 0)
    바깥 반구만 hard         -> 나머지 정점이 아예 안 움직임(간격 597.89um 고정)

full_workflow 의 Y_OFFSET_MM 주석이 적어 둔 실패모드와 같다("soft 는 무반응,
hard 는 충돌 무시"). 그래서 **구속을 전혀 쓰지 않는** 시험으로 바꿨다.

설계 — 중력만으로 강체 바닥에 떨어뜨린다. 구동 기구가 없으니 구동 강성이 결과를
오염시키지 않고, 두 모델에 똑같은 조건이 걸린다. 비교하는 양은 사용자가 지목한
바로 그것들이다:

    충돌 속도 v_in       두 모델이 같은 조건에서 출발했는지(검산)
    최대 접촉력          "몇 N 인가"
    접촉 지속시간        충격 시간 규모
    충격량 ∫F dt         **운동량** 등가성 — mv 변화와 맞아야 한다
    반발계수 e           **에너지** 등가성 — 탄성체는 저장/방출, 점질량은 못 한다

낙하 높이 기본 5mm -> v_in = 0.31 m/s. 분쇄 중 실측 낟알 속력 최대 0.37 m/s 와
같은 규모라 운전 조건을 대변한다.
"""
import os, sys, json
import numpy as np

_r = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_r, "Powder_flip_test"))
sys.path.insert(0, os.path.join(os.path.dirname(_r), "utills"))

DT = float(os.environ.get("DT_MS", "1.0")) * 1e-3     # 충격은 짧다 — 기본 1ms
R = float(os.environ.get("GRAIN_RADIUS_MM", "1.0")) * 1e-3
RHO = float(os.environ.get("GRAIN_RHO", "2160"))
D_HAT = float(os.environ.get("D_HAT", "1e-4"))
NEWTON_TOL = float(os.environ.get("NEWTON_TOL", "1e-4"))
MODEL = os.environ.get("GRAIN_MODEL", "particle").lower()
DROP_MM = float(os.environ.get("DROP_MM", "5.0"))
N_STEP = int(os.environ.get("N_STEP", "400"))
E_PA = float(os.environ.get("E_PA", "4.0e10"))
NU = float(os.environ.get("NU", "0.25"))
SUBDIV = int(os.environ.get("SUBDIV", "2"))


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def sphere_tets(r_mm, subdiv=2):
    """구 tet — icosphere 표면 + 중심 부채꼴(star-shaped 이라 유효, sliver 없음)."""
    import trimesh as tm
    ico = tm.creation.icosphere(subdivisions=subdiv, radius=r_mm)
    v = np.asarray(ico.vertices, float)
    f = np.asarray(ico.faces, np.int64)
    ci = len(v)
    verts = np.vstack([v, [[0.0, 0.0, 0.0]]])
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
    import uipc.core as ucore
    import uipc.geometry as ugeo

    gs.init(backend=gs.gpu, logging_level="warning", precision="64", seed=0)
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=D_HAT, contact_friction_enable=False, two_way_coupling=True,
            enable_rigid_rigid_contact=False, enable_rigid_ground_contact=True,
            newton_tolerance=NEWTON_TOL),
        show_viewer=False)
    coupler = _build_grain_coupler_class()(scene._sim, scene._sim.coupler_options)
    scene._sim._coupler = coupler
    scene.add_entity(gs.morphs.Plane(pos=(0, 0, 0)),
                     material=gs.materials.Rigid(coup_type="ipc_only"))

    m1 = RHO * (4.0 / 3.0) * np.pi * R ** 3
    z0 = R + DROP_MM * 1e-3
    ent = None
    if MODEL == "particle":
        spec = coupler.add_grains(np.array([[0.0, 0.0, z0]]), radius=R,
                                  mass_density=RHO, friction_mu=0.0)
    elif MODEL == "rigid":
        # **사용자 제안 검증(2026-09-29)**: 알을 uipc Particle 대신 **강체 구**로
        # 두면 정량적인 힘이 나오나. 강체는 uipc 에서 ABD 로 들어가는데, 배리어는
        # constitution 이 아니라 contact system 의 속성(전역 d_hat)이라 같은
        # 곡선을 탈 것으로 의심된다 — 그걸 재는 것이다.
        ent = scene.add_entity(
            gs.morphs.Sphere(radius=R, pos=(0.0, 0.0, z0)),
            material=gs.materials.Rigid(rho=RHO, coup_type="ipc_only"))
        spec = None
        print(f"[probe] 강체 구  rho={RHO:.0f}  (uipc ABD 로 등록)")
    else:
        from primitive_tablet_generator import add_analytic_fem_entity
        v_mm, tets = sphere_tets(R * 1e3, SUBDIV)
        mat = gs.materials.FEM.Elastic(E=E_PA, nu=NU, rho=RHO,
                                       model="stable_neohookean")
        ent = add_analytic_fem_entity(
            scene, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "_impact_sphere.stl"),
            v_mm, tets, mat, scale=1e-3, pos=(0.0, 0.0, z0))
        spec = None
        print(f"[probe] 구 tet: 정점 {len(v_mm)}  tet {len(tets)}  E={E_PA:.3g}Pa nu={NU}")
    scene.build(n_envs=0)

    csf = coupler._ipc_world.features().find(ucore.ContactSystemFeature)
    ptypes = list(csf.contact_primitive_types())
    print(f"[probe] MODEL={MODEL}  R={R*1e3:.2f}mm  m={m1*1e6:.4f}mg  "
          f"낙하 {DROP_MM:.1f}mm (v_in 이론 {np.sqrt(2*9.81*DROP_MM*1e-3):.4f} m/s)  "
          f"dt={DT*1e3:.2f}ms  d_hat={D_HAT*1e6:.0f}um  newton_tol={NEWTON_TOL:g}")

    def com():
        if MODEL == "particle":
            return coupler.get_grain_positions(spec)[0]
        if MODEL == "rigid":
            return _npy(ent.get_pos()).squeeze()
        return _npy(ent.get_state().pos).squeeze().mean(axis=0)

    def cforce():
        """이 물체가 접촉으로 받는 합력 z성분 [N]."""
        fz = 0.0
        ni = 0
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
            fz += float((-gr[:, 2] / (DT * DT)).sum())
            ni += n
        return fz, ni

    def cforce_rigid():
        """강체는 접촉 gradient 에 안 잡힌다(ABD 는 12 DOF, 정점이 아니다).
        IPC->강체 커플링 반력(out_forces)이 그 몫이다 — full_workflow 가 링크
        반력을 읽는 것과 같은 채널이다."""
        cd = getattr(scene.sim.coupler, "_coupling_data", None)
        if cd is None:
            return 0.0, 0
        F = cd.out_forces[0]
        names = [l.name for l in cd.links]
        # 바닥(plane)과 구 중 구 쪽 링크를 고른다.
        idx = [i for i, n in enumerate(names) if "plane" not in str(n).lower()]
        if not idx:
            return 0.0, 0
        j = max(idx, key=lambda i: abs(float(F[i][2])))
        return float(F[j][2]), 1

    # 바닥 평면은 IPC 강체라 gradient 에 안 잡힌다(정점이 없다) — 위 합은 낟알 몫이다.
    t, Z, V, F, NI = [], [], [], [], []
    p_prev = com()
    for k in range(N_STEP):
        scene.step()
        p = com()
        v = (p - p_prev) / DT
        fz, ni = cforce_rigid() if MODEL == "rigid" else cforce()
        t.append((k + 1) * DT); Z.append(p[2]); V.append(v[2]); F.append(fz); NI.append(ni)
        p_prev = p
    t, Z, V, F, NI = map(np.asarray, (t, Z, V, F, NI))

    # ── 충격 구간 = 접촉 항목이 있는 연속 구간(첫 번째) ────────────────────
    on = NI > 0
    if not on.any():
        print("[probe] 접촉이 안 잡혔다 — 낙하 높이/스텝수를 늘려라")
        return
    i0 = int(np.argmax(on))
    i1 = i0 + int(np.argmax(~on[i0:])) if (~on[i0:]).any() else len(on) - 1
    v_in = float(V[max(i0 - 1, 0)])
    v_out = float(V[min(i1 + 1, len(V) - 1)])
    imp = float(np.sum(F[i0:i1 + 1]) * DT)                 # ∫F dt  [N·s]
    dp = float(m1 * (v_out - v_in))                        # 운동량 변화
    grav_imp = float(-m1 * 9.81 * (i1 - i0 + 1) * DT)      # 접촉 중 중력 충격량
    print(f"\n[결과] {MODEL}" + (f" E={E_PA:.3g}" if MODEL != "particle" else ""))
    print(f"  충돌 속도 v_in        {v_in*1e3:+9.2f} mm/s")
    print(f"  반발 속도 v_out       {v_out*1e3:+9.2f} mm/s")
    print(f"  반발계수 e            {abs(v_out/v_in) if v_in else np.nan:9.4f}")
    print(f"  최대 접촉력           {F[i0:i1+1].max()*1e3:9.3f} mN "
          f"(= 자중 {m1*9.81*1e3:.4f} mN 의 {F[i0:i1+1].max()/(m1*9.81):.0f} 배)")
    print(f"  접촉 지속             {(i1-i0+1)*DT*1e3:9.2f} ms ({i1-i0+1} 스텝)")
    print(f"  충격량 ∫F dt          {imp*1e6:9.3f} uN·s")
    print(f"  운동량 변화 m·dv      {dp*1e6:9.3f} uN·s")
    print(f"  검산 (∫F dt + 중력) / m·dv = {(imp+grav_imp)/dp if dp else np.nan:.4f}  (1.0 이어야)")
    print(f"  최저 z                {Z[i0:i1+1].min()*1e6:9.2f} um "
          f"(접촉 시작 z {Z[i0]*1e6:.2f} um, 반경 {R*1e6:.0f} um)")
    print(f"  침입량 (R - z_min)    {(R-Z[i0:i1+1].min())*1e6:9.2f} um  "
          f"(d_hat = {D_HAT*1e6:.0f} um)")

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       f"_impact_{MODEL}" + (f"_E{E_PA:.0e}" if MODEL != "particle" else "")
                       + f"_dhat{D_HAT*1e6:.0f}um.json")
    with open(out, "w", encoding="utf-8") as fp:
        json.dump(dict(model=MODEL, E=E_PA, nu=NU, R=R, rho=RHO, m=m1, dt=DT,
                       d_hat=D_HAT, drop_mm=DROP_MM, newton_tol=NEWTON_TOL,
                       v_in=v_in, v_out=v_out, e=abs(v_out / v_in) if v_in else None,
                       F_max=float(F[i0:i1 + 1].max()), dur_s=(i1 - i0 + 1) * DT,
                       impulse=imp, dp=dp, z_min=float(Z[i0:i1 + 1].min()),
                       t=t.tolist(), z=Z.tolist(), v=V.tolist(), F=F.tolist()), fp)
    print(f"[saved] {out}")


if __name__ == "__main__":
    main()

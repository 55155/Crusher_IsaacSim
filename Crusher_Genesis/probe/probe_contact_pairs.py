"""probe_contact_pairs.py — 접촉을 **쌍(i,j) 단위**로 꺼낼 수 있나 (2026-09-23)

사용자 지시: "논문에는 복잡한 수식을 쓸수록 검증절차 또한 복잡해진다. 알별로 id 가
존재하는가? 존재한다면 어떤 식으로 작성할 수 있을까?"

배경. 지금 `full_workflow._contact_forces()` 는 `contact_gradient` 가 주는 **정점별**
gradient 를 알 ID 로 모아 알당 힘을 만든다. 알 ID 자체는 이미 있다
(`i - global_vertex_offset` = `get_grain_positions()` 행 번호). 문제는 모으는 순간
"누가 누구를 밀었나"가 사라진다는 것이다. 그래서 알당 합은 검증식이 길어진다 —
상쇄를 해명해야 하고, 벡터합/크기합/최대값 중 무엇을 쓸지 매번 정당화해야 한다.

쌍으로 꺼낼 수 있으면 표현이 한 줄로 끝난다:
    "알 i 와 알 j 사이에 f_ij [N] 이 걸렸다"
검증도 한 줄이다 — 뉴턴 3법칙 f_ij = -f_ji, 그리고 알별 합이 m(a-g) 와 같은가.
파생량이 없으니 논문에 쓸 수식이 늘지 않는다.

이 프로브가 묻는 것은 하나: **uipc 가 쌍 인덱스를 내주는가.**
  - `contact_gradient(prim_type, G)` -> G.instances() 에 `grad`, `i`  (정점 단위)
  - `contact_hessian(prim_type, G)`  -> ? 헤시안은 블록이 쌍으로 색인되므로
                                        (i, j) 가 있을 **가능성**이 있다.
둘 다 실제로 열어 속성 이름/모양을 전부 찍는다. 추측하지 않고 본다.

씬은 최소로 — 12알을 좁은 기둥에 쌓아 서로 눌리게만 한다. 정적평형 검산이
목적이 아니라 **스키마 확인**이므로 길게 정착시키지 않는다(§27-11 의 4,000스텝은
검산용이었다).
"""
import os, sys
import numpy as np

_r = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_r, "Powder_flip_test"))

DT = 5e-3
N_GRAINS = int(os.environ.get("N_GRAINS", "12"))
R = float(os.environ.get("GRAIN_RADIUS_MM", "1.0")) * 1e-3
RHO = float(os.environ.get("GRAIN_RHO", "1500"))
D_HAT = float(os.environ.get("D_HAT", "1e-4"))
N_SETTLE = int(os.environ.get("N_SETTLE", "300"))


def dump(geo, label):
    """Geometry 안에 실제로 뭐가 들어 있는지 — instances/vertices/meta 전부."""
    print(f"    [{label}]")
    for slot_name in ("instances", "vertices", "meta"):
        try:
            slot = getattr(geo, slot_name)()
        except Exception as e:
            print(f"      {slot_name:10s} 접근 실패 {e!r}")
            continue
        try:
            size = slot.size()
        except Exception:
            size = "-"
        names = []
        for nm in ("i", "j", "hess", "grad", "I", "J", "index", "indices",
                   "pair", "P", "d", "distance"):
            try:
                a = slot.find(nm)
            except Exception:
                a = None
            if a is not None:
                try:
                    v = np.asarray(a.view())
                    names.append(f"{nm}{v.shape}{v.dtype}")
                except Exception:
                    names.append(f"{nm}(view 실패)")
        # 이름을 모르는 속성까지 훑는다 — API 가 목록을 주면 그걸 쓴다.
        for meth in ("attribute_names", "names", "keys"):
            if hasattr(slot, meth):
                try:
                    names.append(f"<{meth}()={list(getattr(slot, meth)())}>")
                    break
                except Exception:
                    pass
        print(f"      {slot_name:10s} size={size}  {' '.join(names) if names else '(속성 못 찾음)'}")


def main():
    import genesis as gs
    from ipc_grain_coupler import _build_grain_coupler_class
    import uipc.core as ucore
    import uipc.geometry as ugeo

    gs.init(backend=gs.gpu, logging_level="warning", precision="32", seed=0)
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=D_HAT, contact_friction_enable=True, two_way_coupling=True,
            enable_rigid_rigid_contact=False, enable_rigid_ground_contact=True),
        show_viewer=False)
    coupler = _build_grain_coupler_class()(scene._sim, scene._sim.coupler_options)
    scene._sim._coupler = coupler
    scene.add_entity(gs.morphs.Plane(pos=(0, 0, 0)),
                     material=gs.materials.Rigid(coup_type="ipc_only"))

    sp = 2 * R + 4 * D_HAT
    pts = np.array([(0.0, 0.0, R + 0.001 + i * sp * 1.02) for i in range(N_GRAINS)])
    spec = coupler.add_grains(pts, radius=R, mass_density=RHO, friction_mu=0.6)
    scene.build(n_envs=0)

    m1 = RHO * (4.0 / 3.0) * np.pi * R ** 3
    print(f"[probe] {N_GRAINS}알 한 줄 기둥  R={R*1e3:.2f}mm  1알 {m1*1e6:.3f}mg  "
          f"1알 자중 {m1*9.81*1e3:.4f} mN  d_hat={D_HAT*1e3:.3f}mm")
    for _ in range(N_SETTLE):
        scene.step()

    world = coupler._ipc_world
    csf = world.features().find(ucore.ContactSystemFeature)
    ptypes = list(csf.contact_primitive_types())
    import uipc as _u
    gg = coupler.grain_slots[(spec, 0)].geometry()
    lo = int(np.asarray(gg.meta().find(_u.builtin.global_vertex_offset).view()).ravel()[0])
    hi = lo + gg.vertices().size()
    print(f"[probe] 낟알 전역 정점 [{lo},{hi})  프리미티브 {ptypes}")

    for pt in ptypes:
        for fn_name in ("contact_gradient", "contact_hessian"):
            G = ugeo.Geometry()
            try:
                getattr(csf, fn_name)(pt, G)
            except Exception as e:
                print(f"  {pt:6s} {fn_name:17s} 호출 실패 {e!r}")
                continue
            try:
                n = G.instances().size()
            except Exception:
                n = -1
            if n <= 0:
                continue
            print(f"  {pt:6s} {fn_name:17s} instances={n}")
            dump(G, f"{pt} {fn_name}")

    # ── 쌍이 나오면 뉴턴 3법칙으로 바로 검증한다 ────────────────────────
    print("\n[probe] PP+N 에서 쌍 복원 시도 — i/j 가 있으면 f_ij = -f_ji 인가")
    G = ugeo.Geometry()
    csf.contact_hessian("PP+N", G)
    inst = G.instances()
    try:
        ii = np.asarray(inst.find("i").view()).ravel()
        jj = np.asarray(inst.find("j").view()).ravel()
        print(f"    i {ii.shape} 범위 [{ii.min()},{ii.max()}]  "
              f"j {jj.shape} 범위 [{jj.min()},{jj.max()}]")
        print(f"    낟알 쌍(둘 다 낟알) 개수 "
              f"{int(((ii>=lo)&(ii<hi)&(jj>=lo)&(jj<hi)).sum())}")
    except Exception as e:
        print(f"    i/j 없음 — {e!r}")
        print("    => 쌍 인덱스는 헤시안으로도 안 나온다. 정점 단위 gradient 만 있다.")

    # ── gradient 행이 접촉별로 짝지어 나오는가 ──────────────────────────
    # 22행 = 11접촉 x 2정점 이면 합쳐진 게 아니라 **접촉별 행**이다. 그렇다면
    # 연속한 두 행이 한 접촉의 양쪽이어야 하고, 뉴턴 3법칙으로 grad 가 정확히
    # 반대여야 한다. 맞으면 쌍 복원에 헤시안조차 필요 없다.
    print("\n[probe] gradient 행이 접촉별 짝인가 — 연속 두 행의 grad 가 반대인가")
    G = ugeo.Geometry()
    csf.contact_gradient("PP+N", G)
    inst = G.instances()
    n = inst.size()
    gr = np.asarray(inst.find("grad").view()).reshape(n, 3)
    ix = np.asarray(inst.find("i").view()).ravel()
    print(f"    행 {n}개  i = {ix.tolist()}")
    ok = 0
    for k in range(0, n - 1, 2):
        s = np.linalg.norm(gr[k] + gr[k + 1])
        m = max(np.linalg.norm(gr[k]), 1e-30)
        if s / m < 1e-9 and ix[k] != ix[k + 1]:
            ok += 1
    print(f"    연속쌍 {n//2}개 중 |g_k+g_k+1|/|g_k| < 1e-9 이고 i 가 다른 것: {ok}")
    if ok == n // 2:
        f = -gr[0::2] / (DT * DT)
        p = coupler.get_grain_positions(spec)
        a, b = ix[0::2], ix[1::2]
        axis = p[a] - p[b]
        axis /= np.maximum(np.linalg.norm(axis, axis=1, keepdims=True), 1e-30)
        algn = np.abs(np.sum(f / np.maximum(np.linalg.norm(f, axis=1, keepdims=True),
                                            1e-30) * axis, axis=1))
        print(f"    => **쌍 복원 성립.** (i,j,f_ij) 로 바로 쓸 수 있다.")
        print(f"    쌍별 |f| [mN]: {np.round(np.linalg.norm(f, axis=1)*1e3, 4).tolist()}")
        print(f"    1알 자중 {m1*9.81*1e3:.4f} mN — 기둥이면 아래로 갈수록 누적이어야 한다")
        print(f"    힘이 두 알을 잇는 축과 평행한가 |cos| 최소 {algn.min():.6f} (PP 는 1.0)")


if __name__ == "__main__":
    main()

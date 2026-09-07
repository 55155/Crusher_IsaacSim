"""낟알 <-> Crusher 벽 커플링 성립 여부만 본다.
가설: (a) 벽이 낟알을 밀어낸다(단방향 성립) (b) 낟알이 벽을 되민다(양방향 성립).
(b)는 '지령 대비 실제 벽 위치의 지연/스톨'로 관측한다."""
import os, sys, numpy as np
sys.path.insert(0, r"C:\Crusher_isaacsim\Crusher_Genesis")
sys.path.insert(0, r"C:\Crusher_isaacsim\utills")
sys.path.insert(0, r"C:\Crusher_isaacsim\Crusher_Genesis\Powder_flip_test")
import paths, genesis as gs
from ipc_grain_coupler import _build_grain_coupler_class
# 원본 MJCF 는 <equality><joint> 때문에 로드가 죽는다 — full_workflow 의
# 준비 함수를 그대로 쓴다(같은 자산/같은 가공을 써야 결과가 이관된다).
sys.path.insert(0, r"C:\Crusher_isaacsim\Crusher_Genesis\Crusher_M0609_RG2_Tablet_Samplebag")
os.environ.setdefault("RUN_TAG", "probe_grain_crusher")
import full_workflow as FW

N       = int(os.environ.get("N_GRAINS", "200"))
WITH_G  = os.environ.get("WITH_GRAINS", "1") == "1"   # 변인통제: 낟알 없는 대조군
DT      = 5e-3
WALL_JOINT = "L1_Guide1_1_L2_Left_Wall1_1"
gs.init(backend=gs.gpu, logging_level="warning", precision="32", seed=0)
GrainIPCCoupler = _build_grain_coupler_class()

scene = gs.Scene(
    sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
    coupler_options=gs.options.IPCCouplerOptions(
        contact_d_hat=1.0e-4, contact_friction_enable=True, two_way_coupling=True,
        enable_rigid_rigid_contact=False, enable_rigid_ground_contact=False,
        constraint_strength_translation=100.0, constraint_strength_rotation=100.0),
    show_viewer=False)
coupler = GrainIPCCoupler(scene._sim, scene._sim.coupler_options)
scene._sim._coupler = coupler

scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(coup_type="ipc_only"))
crusher = scene.add_entity(
    gs.morphs.MJCF(file=FW._prepare_crusher_mjcf(), pos=FW.CRUSHER_POS, euler=FW.CRUSHER_EULER,
                   decimate=True, convexify=True),
    material=gs.materials.Rigid(coup_type="two_way_soft_constraint", coup_friction=0.8))
print("[probe] Crusher 추가 OK")

spec = None
if WITH_G:
    # 슬롯 안(벽 앞)에 낟알을 흩뿌린다. 좌표는 아래 슬롯 계측 후 확정하므로 임시.
    rng = np.random.default_rng(0)
    # **슬롯 위 빈 공간**에서 떨어뜨린다. 슬롯 안에 직접 놓으면 벽/본체와 겹쳐
    # uipc 빌드가 죽는다(AffineBodyStateAccessorFeature=None -> body_count).
    # **격자 배치 필수** — 난수로 뿌리면 서로 겹쳐 libuipc 초기 거리검사에 걸린다
    # ("World is not valid, skipping init" -> ABD 피처 미등록 -> body_count).
    sx, sy, sz = 0.2125, -0.0053, 0.150      # 슬롯 중심 위
    d = 0.005
    pos, k = [], 0
    while len(pos) < N:
        i, j = k % 3, (k // 3) % 3
        pos.append((sx + (i-1)*d, sy + (j-1)*d, sz + (k // 9) * d))
        k += 1
    pos = np.array(pos[:N])
    spec = coupler.add_grains(pos, radius=1.5e-3, mass_density=1500.0, friction_mu=0.6)
scene.build(n_envs=0)
print("[probe] build OK  coupler =", type(scene.sim.coupler).__name__)

_cj = {j.name: j for j in crusher.joints}
_d = _cj[WALL_JOINT].dofs_idx_local
wall_dof = _d[0] if isinstance(_d, (list, tuple, np.ndarray)) else _d
crusher.set_dofs_kp(np.array([5000.0]), dofs_idx_local=[wall_dof])
crusher.set_dofs_kv(np.array([500.0]),  dofs_idx_local=[wall_dof])
crusher.set_dofs_force_range(np.array([-100.0]), np.array([100.0]), dofs_idx_local=[wall_dof])
_np = lambda x: x.detach().cpu().numpy() if hasattr(x, "detach") else np.asarray(x)
G = (lambda: coupler.get_grain_positions(spec)) if WITH_G else (lambda: None)

# 1) 낙하/정착 — 낟알이 슬롯 안으로 들어가게 둔다.
if WITH_G:
    for k in range(600):
        crusher.control_dofs_position(np.array([0.006]), dofs_idx_local=[wall_dof])
        scene.step()
        if k % 200 == 0 or k == 599:
            p = G(); print(f"[probe] 낙하 k={k:4d} z최저 {p[:,2].min()*1e3:6.1f} "
                           f"z평균 {p[:,2].mean()*1e3:6.1f}mm  x폭 {(p[:,0].max()-p[:,0].min())*1e3:5.1f}mm")
    _p0 = G().copy()

# 2) 벽 닫기 — full_workflow 와 같은 속도지령 방식.
q = 0.006; n = int(round(0.021 / 0.008 / DT))
print(f"[probe] 벽 +6.0 -> -15.0mm, {n}스텝  낟알={'ON' if WITH_G else 'OFF'}({N if WITH_G else 0}개)")
for k in range(n):
    q = max(q - 0.008 * DT, -0.015)
    crusher.control_dofs_position(np.array([q]), dofs_idx_local=[wall_dof])
    scene.step()
    if k % 200 == 0 or k == n - 1:
        act = float(_np(crusher.get_dofs_position())[wall_dof])
        s = ""
        if WITH_G:
            p = G(); s = f" | 낟알 x[{p[:,0].min()*1e3:6.1f},{p[:,0].max()*1e3:6.1f}] z최저 {p[:,2].min()*1e3:5.1f}mm"
        print(f"[probe] k={k:5d} cmd={q*1e3:+7.2f} act={act*1e3:+7.2f} 지연={(q-act)*1e3:+6.2f}mm{s}")

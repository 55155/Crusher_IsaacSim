"""
open_crusher.py
===============
Crusher_IsaacSim.xml 을 mujoco_playground 로 엽니다.

실행:
  conda activate isaacsim
  python open_crusher.py
"""

import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="mujoco")

import jax
import jax.numpy as jnp
import mujoco
import mujoco.viewer
from mujoco import mjx
from ml_collections import config_dict
from mujoco_playground import MjxEnv, State

MJCF_PATH = r"C:\Crusher_isaacsim\MuJoCo_PlayGround\MJCF\Crusher_IsaacSim.xml"


class CrusherEnv(MjxEnv):

    def __init__(self):
        cfg = config_dict.create(ctrl_dt=0.02, sim_dt=0.002)
        super().__init__(cfg)
        self._mj_model  = mujoco.MjModel.from_xml_path(MJCF_PATH)
        self._mjx_model = mjx.put_model(self._mj_model)

    @property
    def xml_path(self) -> str:
        return MJCF_PATH

    @property
    def mj_model(self) -> mujoco.MjModel:
        return self._mj_model

    @property
    def mjx_model(self) -> mjx.Model:
        return self._mjx_model

    @property
    def action_size(self) -> int:
        return self._mj_model.nu

    def reset(self, rng: jax.Array) -> State:
        data = mjx.make_data(self._mjx_model)
        obs  = jnp.zeros(self._mj_model.nq)
        return State(data, obs,
                     reward=jnp.float32(0.0),
                     done=jnp.bool_(False),
                     metrics={}, info={})

    def step(self, state: State, action: jax.Array) -> State:
        data = mjx.step(self._mjx_model, state.data)
        obs  = jnp.zeros(self._mj_model.nq)
        return state.replace(data=data, obs=obs)


# ── 실행 ──────────────────────────────────────────────────────────────────────
env   = CrusherEnv()
state = jax.jit(env.reset)(jax.random.PRNGKey(0))

print(f"bodies  : {env.mj_model.nbody}")
print(f"joints  : {env.mj_model.njnt}")
print(f"geoms   : {env.mj_model.ngeom}")
print(f"actuator: {env.mj_model.nu}")

print("\nmujoco viewer 시작 (Space: 일시정지, ESC: 종료)")
mujoco.viewer.launch(env.mj_model, mujoco.MjData(env.mj_model))

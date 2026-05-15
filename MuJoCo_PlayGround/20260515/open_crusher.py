"""
open_crusher.py
===============
Crusher_IsaacSim.xml 을 mujoco_playground 로 엽니다.

실행:
  conda activate isaacsim
  python open_crusher.py
"""

import jax
import jax.numpy as jnp
import mujoco
from mujoco import mjx
from mujoco_playground import MjxEnv, State

MJCF_PATH = r"C:\Crusher_isaacsim\MuJoCo_PlayGround\MJCF\Crusher_IsaacSim.xml"


class CrusherEnv(MjxEnv):

    def __init__(self):
        model = mujoco.MjModel.from_xml_path(MJCF_PATH)
        super().__init__(model)

    def reset(self, rng: jax.Array) -> State:
        data = mjx.make_data(self.model)
        obs  = jnp.zeros(self.observation_size)
        return State(data, obs,
                     reward=jnp.float32(0.0),
                     done=jnp.bool_(False),
                     metrics={}, info={})

    def step(self, state: State, action: jax.Array) -> State:
        data = mjx.step(self.model, state.data)
        obs  = jnp.zeros(self.observation_size)
        return state.replace(data=data, obs=obs)

    @property
    def observation_size(self) -> int:
        return self.model.nq

    @property
    def action_size(self) -> int:
        return self.model.nu


# ── 실행 ──────────────────────────────────────────────────────────────────────
env   = CrusherEnv()
state = jax.jit(env.reset)(jax.random.PRNGKey(0))

print(f"bodies  : {env.model.nbody}")
print(f"joints  : {env.model.njnt}")
print(f"geoms   : {env.model.ngeom}")
print(f"qpos    : {env.model.nq}")
print(f"actuator: {env.model.nu}")

# mujoco 기본 뷰어로 오픈 (interactive)
print("\nmujoco viewer 시작 (Space: 일시정지, ESC: 종료)")
mujoco.viewer.launch(env.mj_model, mujoco.MjData(env.mj_model))

"""
view_mjcf.py
============
Crusher_IsaacSim.xml 을 mujoco_playground 로 로드합니다.

모드 A (기본) : mjx-viewer  →  브라우저에서 http://localhost:5173 접속
모드 B        : render_array →  PNG 프레임 저장

실행:
  conda activate isaacsim
  python view_mjcf.py          # 모드 A (브라우저 뷰어)
  python view_mjcf.py --render  # 모드 B (PNG 저장)
"""

import sys
import subprocess
import pathlib

import jax
import jax.numpy as jnp
import mujoco
from mujoco import mjx
from mujoco_playground import MjxEnv, State, render_array

MJCF_PATH   = r"C:\Crusher_isaacsim\MuJoCo_PlayGround\MJCF\Crusher_IsaacSim.xml"
OUTPUT_DIR  = pathlib.Path(r"C:\Crusher_isaacsim\MuJoCo_PlayGround\20260514\frames")
MJX_VIEWER  = r"C:\Users\simuser\anaconda3\envs\isaacsim\Scripts\mjx-viewer.exe"


# ── CrusherEnv 정의 ────────────────────────────────────────────────────────────
class CrusherEnv(MjxEnv):
    def __init__(self):
        mj_model = mujoco.MjModel.from_xml_path(MJCF_PATH)
        mj_model.opt.timestep = 0.002
        super().__init__(mj_model)

    def reset(self, rng: jax.Array) -> State:
        data = mjx.make_data(self.model)
        data = data.replace(qpos=self.model.qpos0)
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


# ── 환경 초기화 ────────────────────────────────────────────────────────────────
print("CrusherEnv 초기화 중...")
env   = CrusherEnv()
rng   = jax.random.PRNGKey(0)
state = jax.jit(env.reset)(rng)

print(f"  bodies  : {env.model.nbody}")
print(f"  joints  : {env.model.njnt}")
print(f"  geoms   : {env.model.ngeom}")
print(f"  actuator: {env.model.nu}")
print("  reset 완료\n")


# ── 모드 B: render_array → PNG 저장 ──────────────────────────────────────────
def mode_render(n_steps: int = 60):
    from PIL import Image
    import numpy as np

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    step_fn = jax.jit(env.step)

    trajectory = [state]
    s = state
    for _ in range(n_steps):
        action = jnp.zeros(env.action_size)
        s = step_fn(s, action)
        trajectory.append(s)

    print(f"렌더링 중 ({n_steps} 프레임)...")
    frames = render_array(env.model, trajectory, height=480, width=640)

    for i, frame in enumerate(frames):
        img = Image.fromarray(np.array(frame))
        path = OUTPUT_DIR / f"frame_{i:04d}.png"
        img.save(path)

    print(f"저장 완료 → {OUTPUT_DIR}  ({len(frames)}개)")


# ── 모드 A: mjx-viewer (브라우저) ─────────────────────────────────────────────
def mode_viewer():
    print("mjx-viewer 시작 중...")
    print("브라우저에서 http://localhost:5173 접속\n")
    subprocess.run([MJX_VIEWER, "--mjcf", MJCF_PATH])


# ── 진입점 ─────────────────────────────────────────────────────────────────────
if "--render" in sys.argv:
    mode_render()
else:
    mode_viewer()

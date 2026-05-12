import mujoco
print("MuJoCo version:", mujoco.__version__)
spec = mujoco.MjSpec.from_file(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
spec.compiler.balanceinertia = True
model = spec.compile()
print("compile success! nbody:", model.nbody)

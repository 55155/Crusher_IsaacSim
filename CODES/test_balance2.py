import mujoco
spec = mujoco.MjSpec.from_file(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
spec.compiler.balanceinertia = True
model = spec.compile()
print("nbody:", model.nbody)
print("njnt:", model.njnt)
print("ngeom:", model.ngeom)
print("bodies:", [model.body(i).name for i in range(model.nbody)])

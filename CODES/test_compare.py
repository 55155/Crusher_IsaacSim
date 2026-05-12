import mujoco

# ?? ??
try:
    model = mujoco.MjModel.from_xml_path(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
    print("from_xml_path nbody:", model.nbody)
except Exception as e:
    print("from_xml_path error:", e)

# MjSpec ??
try:
    spec = mujoco.MjSpec.from_file(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
    spec.compiler.balanceinertia = True
    model2 = spec.compile()
    print("MjSpec nbody:", model2.nbody)
except Exception as e:
    print("MjSpec error:", e)

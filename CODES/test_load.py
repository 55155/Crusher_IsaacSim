import mujoco
model = mujoco.MjModel.from_xml_path(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
print("success! nbody:", model.nbody)

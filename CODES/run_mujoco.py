import mujoco, mujoco.viewer
model = mujoco.MjModel.from_xml_path(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
data = mujoco.MjData(model)
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

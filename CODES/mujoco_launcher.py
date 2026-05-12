import sys, mujoco, mujoco.viewer

if len(sys.argv) > 1:
    model = mujoco.MjModel.from_xml_path(sys.argv[1])
    data  = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data) as v:
        while v.is_running():
            mujoco.mj_step(model, data)
            v.sync()
else:
    mujoco.viewer.launch()

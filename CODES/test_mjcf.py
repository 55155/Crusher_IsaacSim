import mujoco

# URDF? MJCF XML ???? ?? ? balanceinertia ??
spec = mujoco.MjSpec.from_file(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
mjcf_str = spec.to_xml()

# compiler ??? balanceinertia ??
if "<compiler" in mjcf_str:
    mjcf_str = mjcf_str.replace("<compiler", "<compiler balanceinertia=\"true\"", 1)
else:
    mjcf_str = mjcf_str.replace("<mujoco", "<mujoco>\n  <compiler balanceinertia=\"true\"/", 1)

# ??
with open(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.xml", "w") as f:
    f.write(mjcf_str)

# ?? ???
model = mujoco.MjModel.from_xml_path(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.xml")
print("nbody:", model.nbody)
print("success!")

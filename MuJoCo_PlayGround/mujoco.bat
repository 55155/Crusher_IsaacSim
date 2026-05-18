@echo off
call C:\Users\simuser\anaconda3\Scripts\activate.bat isaacsim
python -m mujoco.viewer --mjcf C:\Crusher_isaacsim\MuJoCo_PlayGround\MJCF\Crusher_IsaacSim_colored.xml

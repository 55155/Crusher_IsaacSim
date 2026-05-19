@echo off
cd /d "%~dp0"

call conda activate isaac_sim 2>nul
if %errorlevel% neq 0 (
    if exist "%USERPROFILE%\anaconda3\Scripts\activate.bat" (
        call "%USERPROFILE%\anaconda3\Scripts\activate.bat" isaac_sim
    )
)

python -m mujoco.viewer --mjcf MJCF\Crusher_IsaacSim_colored.xml
pause

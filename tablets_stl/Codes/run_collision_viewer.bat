@echo off
chcp 437 > nul
title Collision Mesh Viewer
cd /d "%~dp0"

echo Activating conda environment: isaac_sim
call conda activate isaac_sim 2>nul
if %errorlevel% neq 0 (
    if exist "%USERPROFILE%\anaconda3\Scripts\activate.bat" (
        call "%USERPROFILE%\anaconda3\Scripts\activate.bat" isaac_sim
        goto :check_env
    )
    if exist "C:\ProgramData\Anaconda3\Scripts\activate.bat" (
        call "C:\ProgramData\Anaconda3\Scripts\activate.bat" isaac_sim
        goto :check_env
    )
    echo [WARN] Running with current Python (base).
)

:check_env
echo Active Python: & python --version

echo Installing packages if needed...
python -m pip install pyvista pyvistaqt PyQt5 trimesh numpy --quiet

echo.
echo  [Optional] For higher quality decomposition:
echo  pip install coacd
echo.

echo Launching Collision Mesh Viewer...
python collision_viewer.py %*

pause

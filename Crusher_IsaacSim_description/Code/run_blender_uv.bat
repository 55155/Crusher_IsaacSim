@echo off
setlocal
set BLENDER="C:\Program Files\Blender Foundation\Blender 5.1\blender.exe"
set SCRIPT=%~dp0blender_uv_unwrap.py

echo ============================================================
echo  Crusher IsaacSim - Blender UV Unwrap ^& USD Export
echo ============================================================
echo Blender : %BLENDER%
echo Script  : %SCRIPT%
echo Output  : C:\TEMP\Crusher_IsaacSim_description\meshes_usd\
echo.

if not exist %BLENDER% (
    echo ERROR: Blender not found at %BLENDER%
    echo Please update the BLENDER variable in this bat file.
    pause
    exit /b 1
)

%BLENDER% --background --python "%SCRIPT%"

echo.
echo ============================================================
echo  Done. Check C:\TEMP\Crusher_IsaacSim_description\meshes_usd\
echo ============================================================
pause

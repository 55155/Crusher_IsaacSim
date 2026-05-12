@echo off
setlocal
set PYTHON=C:\Users\simuser\anaconda3\python.exe
set SCRIPT=%~dp0apply_triplanar.py

echo ============================================================
echo  Crusher IsaacSim - OmniPBR Triplanar 재질 적용
echo ============================================================
echo.

if not exist "%PYTHON%" (
    echo ERROR: Python not found at %PYTHON%
    pause & exit /b 1
)

"%PYTHON%" "%SCRIPT%"

echo.
pause

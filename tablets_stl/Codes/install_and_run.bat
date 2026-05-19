@echo off
chcp 437 > nul
title Tablet Viewer
cd /d "%~dp0"

:: ── conda 환경 활성화 ─────────────────────────────────────────
:: bat 파일에서는 반드시 "call conda activate" 를 사용해야 함
:: (conda activate 단독 사용시 실패)

echo Activating conda environment: isaac_sim

call conda activate isaac_sim 2>nul
if %errorlevel% neq 0 (
    echo [WARN] isaac_sim env not found. Trying common Anaconda paths...

    :: 경로 1: 사용자 폴더 Anaconda3
    if exist "%USERPROFILE%\anaconda3\Scripts\activate.bat" (
        call "%USERPROFILE%\anaconda3\Scripts\activate.bat" isaac_sim
        goto :check_env
    )

    :: 경로 2: C:\ProgramData\Anaconda3
    if exist "C:\ProgramData\Anaconda3\Scripts\activate.bat" (
        call "C:\ProgramData\Anaconda3\Scripts\activate.bat" isaac_sim
        goto :check_env
    )

    :: 경로 3: base 환경으로 대체 실행
    echo [WARN] Could not find isaac_sim. Running with current Python (base).
)

:check_env
echo Active Python: & python --version

:: ── 패키지 설치 (없으면 설치) ────────────────────────────────
echo Installing packages if needed...
python -m pip install pyvista pyvistaqt PyQt5 trimesh numpy --quiet

:: ── 뷰어 실행 ────────────────────────────────────────────────
echo Launching Tablet Shape Viewer...
python viewer_desktop.py

pause

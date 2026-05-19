@echo off
title Tablet Shape Viewer
cd /d "%~dp0"

echo.
echo  ╔══════════════════════════════════════╗
echo  ║      Tablet Shape Viewer             ║
echo  ║      http://localhost:8765           ║
echo  ╚══════════════════════════════════════╝
echo.
echo  서버를 시작합니다...

:: Python 서버를 새 창에서 실행
start "Tablet Server" cmd /k "python -m http.server 8765"

:: 서버 기동 대기 (1초)
ping -n 2 127.0.0.1 > nul

:: 브라우저 자동 실행
start "" "http://localhost:8765/viewer.html"

echo  브라우저가 열렸습니다.
echo  서버를 중지하려면 [Tablet Server] 창을 닫으세요.
echo.
pause

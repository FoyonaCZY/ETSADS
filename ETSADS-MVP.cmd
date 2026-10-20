@echo off
setlocal
cd /d "%~dp0"
if not exist "%~dp0target\debug\etsads.exe" (
    echo MVP executable is missing. Build with: cargo build -p etsads -p etsads-supervisor --features etsads/ui
    pause
    exit /b 1
)
if not exist "%~dp0target\debug\etsads-supervisor.exe" (
    echo Safety supervisor is missing. Build with: cargo build -p etsads-supervisor
    pause
    exit /b 1
)
start "" "%~dp0target\debug\etsads.exe"

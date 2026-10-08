@echo off
rem ============================================================
rem Revo2 touch hand GUI - one-click launcher (double click me)
rem All Chinese hints are printed by launcher.py (UTF-8 safe).
rem ============================================================
cd /d "%~dp0"

where python >nul 2>nul
if errorlevel 1 (
  echo [ERROR] Python not found in PATH.
  echo Please install Python 3.10 or newer from https://www.python.org/downloads/
  echo and CHECK the box "Add Python to PATH" during install.
  pause
  exit /b 1
)

python launcher.py

echo.
echo (window closed)
pause

@echo off
cd /d "%~dp0"
echo ===============================================================
echo Starting PAROL6 Native Control Suite & 3D Pipeline Verifier...
echo ===============================================================
if exist "%~dp0bin\Release\net8.0-windows\PAROLC_ControlSuite.exe" (
    start "" "%~dp0bin\Release\net8.0-windows\PAROLC_ControlSuite.exe"
) else if exist "%~dp0bin\Release\PAROLC_ControlSuite.exe" (
    start "" "%~dp0bin\Release\PAROLC_ControlSuite.exe"
) else (
    start "" "%~dp0bin\Debug\net8.0-windows\PAROLC_ControlSuite.exe"
)

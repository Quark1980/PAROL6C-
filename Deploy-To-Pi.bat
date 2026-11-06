@echo off
title PAROL6 Automated Deployment to Raspberry Pi
color 0B
echo ==============================================================================
echo   🤖 PAROL6 ROBOTICS: ONE-CLICK AUTOMATED DEPLOYMENT TOOL
echo ==============================================================================
echo.
echo Launching cross-deployment pipeline...
echo.

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0deploy\deploy_robot.ps1"

if %ERRORLEVEL% NEQ 0 (
    echo.
    color 0C
    echo [ERROR] Deployment failed with error code %ERRORLEVEL%.
    echo Please review the logs above.
) else (
    echo.
    color 0A
    echo [SUCCESS] PAROL6 deployment completed successfully!
)

echo.
pause

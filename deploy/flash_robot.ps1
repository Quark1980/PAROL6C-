<#
.SYNOPSIS
    Automated Cross-Deployment Pipeline: Compiles AVR Firmware, Transfers to Raspberry Pi 5,
    Flashes ATmega2560 on RAMPS 1.4, and Registers the Host Control Daemon.

.DESCRIPTION
    1. Invokes arduino-cli to compile firmware/parol6_core for arduino:avr:mega.
    2. SCPs the compiled .hex binary and daemon files to the Raspberry Pi over Wi-Fi.
    3. Remotely halts any Klipper/Moonraker processes holding /dev/ttyACM0.
    4. Executes avrdude to flash the Mega 2560.
    5. Installs and starts the persistent systemd service (parol6-daemon.service).

.EXAMPLE
    .\deploy\flash_robot.ps1 -PiHost 192.168.1.150 -PiUser pi
#>

[CmdletBinding()]
param (
    [Parameter(Mandatory=$false)]
    [string]$PiHost = "raspberrypi.local",

    [Parameter(Mandatory=$false)]
    [string]$PiUser = "pi",

    [Parameter(Mandatory=$false)]
    [int]$PiPort = 22,

    [Parameter(Mandatory=$false)]
    [string]$SerialDevice = "/dev/ttyACM0",

    [Parameter(Mandatory=$false)]
    [switch]$SkipCompile
)

$ErrorActionPreference = "Stop"

Write-Host "============================================================" -ForegroundColor Cyan
Write-Host " 🤖 PAROL6 ROBOTICS CROSS-DEPLOYMENT PIPELINE" -ForegroundColor Cyan
Write-Host " Target: $PiUser@$PiHost : Port $PiPort" -ForegroundColor Cyan
Write-Host "============================================================" -ForegroundColor Cyan

$WorkspaceRoot = (Get-Item $PSScriptRoot).Parent.FullName
$FirmwareDir = Join-Path $WorkspaceRoot "firmware\parol6_core"
$BuildDir = Join-Path $WorkspaceRoot "firmware\build"
$HexFile = Join-Path $BuildDir "parol6_core.ino.hex"
$DaemonDir = Join-Path $WorkspaceRoot "daemon"
$UiDir = Join-Path $WorkspaceRoot "ui"
$ServiceFile = Join-Path $WorkspaceRoot "deploy\parol6-daemon.service"

# ----------------------------------------------------------------------
# 1. COMPILE AVR FIRMWARE WITH ARDUINO-CLI
# ----------------------------------------------------------------------
if (-not $SkipCompile) {
    Write-Host "`n[1/5] Locating Arduino-CLI Compiler..." -ForegroundColor Yellow

    $ArduinoCli = Get-Command "arduino-cli" -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source -ErrorAction SilentlyContinue
    if (-not $ArduinoCli) {
        $Candidates = @(
            "$env:LOCALAPPDATA\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
            "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
            "C:\Program Files (x86)\Arduino\arduino-cli.exe"
        )
        foreach ($c in $Candidates) {
            if (Test-Path $c) {
                $ArduinoCli = $c
                break
            }
        }
    }

    if (-not $ArduinoCli) {
        Write-Error "arduino-cli.exe could not be found! Please install Arduino IDE or add arduino-cli to your PATH."
    }

    Write-Host "Found compiler at: $ArduinoCli" -ForegroundColor Green
    Write-Host "Compiling firmware for target 'arduino:avr:mega'..." -ForegroundColor Yellow

    if (-not (Test-Path $BuildDir)) {
        New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
    }

    $ConfigDir = "$env:LOCALAPPDATA\Arduino15"
    $CompileArgs = @(
        "--config-dir", $ConfigDir,
        "compile",
        "--fqbn", "arduino:avr:mega",
        "--output-dir", $BuildDir,
        $FirmwareDir
    )

    & $ArduinoCli @CompileArgs
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Firmware compilation failed!"
    }

    if (-not (Test-Path $HexFile)) {
        Write-Error "Compiled hex binary not found at: $HexFile"
    }

    Write-Host "✓ Compilation successful: $(Get-Item $HexFile | Select-Object -ExpandProperty Length) bytes" -ForegroundColor Green
} else {
    Write-Host "`n[1/5] Skipping compilation step as requested." -ForegroundColor DarkGray
}

# ----------------------------------------------------------------------
# 2. CREATE REMOTE DIRECTORY STRUCTURE ON RASPBERRY PI
# ----------------------------------------------------------------------
Write-Host "`n[2/5] Creating remote deployment directories on Raspberry Pi..." -ForegroundColor Yellow
$RemoteRoot = "/home/$PiUser/parol6"

$SshCmd = "ssh -p $PiPort ${PiUser}@${PiHost} 'mkdir -p $RemoteRoot/daemon $RemoteRoot/ui $RemoteRoot/build'"
Invoke-Expression $SshCmd
if ($LASTEXITCODE -ne 0) {
    Write-Error "SSH connection to ${PiUser}@${PiHost} failed. Check network connection and SSH keys."
}

# ----------------------------------------------------------------------
# 3. SCP BINARY & DAEMON FILES ACROSS WI-FI
# ----------------------------------------------------------------------
Write-Host "`n[3/5] Syncing files over Wi-Fi..." -ForegroundColor Yellow

Write-Host " - Transferring compiled hex binary..."
scp -P $PiPort $HexFile "${PiUser}@${PiHost}:${RemoteRoot}/parol6_core.ino.hex"

Write-Host " - Transferring Python daemon modules..."
scp -P $PiPort -r "$DaemonDir\*" "${PiUser}@${PiHost}:${RemoteRoot}/daemon/"

Write-Host " - Transferring Touchscreen Kiosk UI..."
scp -P $PiPort -r "$UiDir\*" "${PiUser}@${PiHost}:${RemoteRoot}/ui/"

Write-Host " - Transferring systemd service & kiosk descriptors..."
scp -P $PiPort $ServiceFile "${PiUser}@${PiHost}:${RemoteRoot}/parol6-daemon.service"
scp -P $PiPort (Join-Path $WorkspaceRoot "deploy\parol6-kiosk.service") "${PiUser}@${PiHost}:${RemoteRoot}/parol6-kiosk.service"
scp -P $PiPort (Join-Path $WorkspaceRoot "deploy\launch-kiosk.sh") "${PiUser}@${PiHost}:${RemoteRoot}/deploy/launch-kiosk.sh"
scp -P $PiPort (Join-Path $WorkspaceRoot "deploy\remote_install.sh") "${PiUser}@${PiHost}:${RemoteRoot}/deploy/remote_install.sh"

Write-Host "✓ File transfer complete." -ForegroundColor Green

# ----------------------------------------------------------------------
# 4. REMOTE HARDWARE FLASH & SYSTEM SETUP
# ----------------------------------------------------------------------
Write-Host "`n[4/5] Executing remote installer, hardware flash & kiosk setup..." -ForegroundColor Yellow

$RemoteExecScript = @"
chmod +x ${RemoteRoot}/deploy/remote_install.sh
bash ${RemoteRoot}/deploy/remote_install.sh ${RemoteRoot} ${SerialDevice}
"@

$EncodedExec = [Convert]::ToBase64String([System.Text.Encoding]::UTF8.GetBytes($RemoteExecScript))
ssh -p $PiPort "${PiUser}@${PiHost}" "echo $EncodedExec | base64 -d | bash"

if ($LASTEXITCODE -ne 0) {
    Write-Error "Remote setup failed! Check output above."
}

# ----------------------------------------------------------------------
# 5. VERIFY SERVICE STATUS
# ----------------------------------------------------------------------
Write-Host "`n[5/5] Verifying service status..." -ForegroundColor Yellow

ssh -p $PiPort "${PiUser}@${PiHost}" "sudo systemctl is-active parol6-daemon.service && sudo systemctl is-active parol6-kiosk.service"

Write-Host "`n============================================================" -ForegroundColor Green
Write-Host " [SUCCESS] PAROL6 DEPLOYMENT COMPLETE & KIOSK READY ON BOOT!" -ForegroundColor Green
Write-Host " WebSocket API:  ws://${PiHost}:8765" -ForegroundColor Green
Write-Host " Web Interface:  http://${PiHost}:8000" -ForegroundColor Green
Write-Host " Kiosk Mode:     Auto-starts on Pi boot at http://localhost:8000" -ForegroundColor Green
Write-Host "============================================================" -ForegroundColor Green

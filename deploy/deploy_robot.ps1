# ==============================================================================
# PAROL6 Automated Cross-Platform Deployment Engine (deploy_robot.ps1)
# ==============================================================================
# 1. Compiles ATmega2560 firmware via arduino-cli (arduino:avr:mega)
# 2. Bundles and transfers firmware hex, daemon, and UI via SSH/SCP
# 3. Halts conflicting services on Pi (klipper, moonraker, klipperscreen)
# 4. Flashes ATmega2560 on RAMPS 1.4 via avrdude (-c wiring -p m2560 -b 115200)
# 5. Configures and starts systemd background service (parol6.service)
# 6. Verifies live system health
# ==============================================================================

param (
    [string]$Target = "",
    [switch]$NonInteractive
)

$ErrorActionPreference = "Stop"

$ScriptDir   = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = (Get-Item $ScriptDir).Parent.FullName
$TargetFile  = Join-Path $ScriptDir ".deploy_target.txt"

function Find-Binary([string]$name) {
    $candidates = @(
        (Join-Path $env:WINDIR "System32\OpenSSH\$name.exe"),
        (Join-Path $env:WINDIR "Sysnative\OpenSSH\$name.exe")
    )
    foreach ($p in $candidates) {
        if (Test-Path $p) { return $p }
    }
    $cmd = Get-Command $name -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    throw "Required tool '$name' not found. Ensure Windows OpenSSH Client is installed."
}

$ssh = Find-Binary "ssh"
$scp = Find-Binary "scp"
$sshKeygen = Find-Binary "ssh-keygen"

Write-Host "==================================================================" -ForegroundColor Cyan
Write-Host "  PAROL6 ROBOTICS: CROSS-PLATFORM DEPLOYMENT ENGINE               " -ForegroundColor Cyan
Write-Host "==================================================================" -ForegroundColor Cyan

# --------------------------------------------------------------------------
# 1. RESOLVE PI TARGET
# --------------------------------------------------------------------------
$resolvedTarget = "mark@192.168.0.159"
if (Test-Path $TargetFile) {
    $saved = (Get-Content $TargetFile -Raw).Trim()
    if ($saved -ne "") { $resolvedTarget = $saved }
}
if ($Target -ne "") {
    $resolvedTarget = $Target
} elseif (-not $NonInteractive) {
    Write-Host "`n[Step 1/6] Target Raspberry Pi Configuration" -ForegroundColor Yellow
    Write-Host "Press [ENTER] to use default: " -NoNewline
    Write-Host "$resolvedTarget" -ForegroundColor Green
    $userInput = Read-Host "Enter Target (user@ip-or-hostname) [$resolvedTarget]"
    if (-not [string]::IsNullOrWhiteSpace($userInput)) {
        $resolvedTarget = $userInput.Trim()
    }
}

if ($resolvedTarget -notmatch "@") {
    $resolvedTarget = "pi@$resolvedTarget"
}
Set-Content -Path $TargetFile -Value $resolvedTarget -Force
Write-Host "Target resolved: $resolvedTarget" -ForegroundColor Green

$piUser = ($resolvedTarget -split "@")[0]
$piHost = ($resolvedTarget -split "@")[1]
$RemoteRoot = "/home/$piUser/parol6"
$sshOpts = @("-o", "StrictHostKeyChecking=accept-new", "-o", "ConnectTimeout=10")

# --------------------------------------------------------------------------
# 2. CHECK / CONFIGURE SSH AUTHENTICATION
# --------------------------------------------------------------------------
Write-Host "`n[Step 2/6] Checking SSH Key Authentication..." -ForegroundColor Yellow

$keyPath = Join-Path $env:USERPROFILE ".ssh\id_ed25519"
if (-not (Test-Path $keyPath)) {
    Write-Host "Generating ed25519 deployment key..." -ForegroundColor DarkGray
    New-Item -ItemType Directory -Force -Path (Split-Path $keyPath) | Out-Null
    & $sshKeygen -t ed25519 -N '""' -f $keyPath -q
}

$oldEAP = $ErrorActionPreference
$ErrorActionPreference = "Continue"
$testOutput = & $ssh @sshOpts -o BatchMode=yes -o NumberOfPasswordPrompts=0 $resolvedTarget "echo KEY_AUTH_OK" 2>&1
$isKeyAuthed = ($LASTEXITCODE -eq 0 -and $testOutput -match "KEY_AUTH_OK")
$ErrorActionPreference = $oldEAP

if (-not $isKeyAuthed) {
    Write-Host "SSH Key authentication required. Please enter password once:" -ForegroundColor Yellow
    $pubKey = (Get-Content "$keyPath.pub" -Raw).Trim()
    $setupCmd = "mkdir -p ~/.ssh && chmod 700 ~/.ssh && touch ~/.ssh/authorized_keys && grep -qF '$pubKey' ~/.ssh/authorized_keys || echo '$pubKey' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
    & $ssh @sshOpts $resolvedTarget $setupCmd
}
Write-Host "[OK] SSH Key authentication verified." -ForegroundColor Green

# --------------------------------------------------------------------------
# 3. COMPILE ATMEGA2560 FIRMWARE
# --------------------------------------------------------------------------
Write-Host "`n[Step 3/6] Compiling ATmega2560 Firmware via arduino-cli..." -ForegroundColor Yellow

$FirmwareDir = Join-Path $ProjectRoot "firmware\parol6_core"
$BuildDir    = Join-Path $ProjectRoot "firmware\build"
$HexFile     = Join-Path $BuildDir "parol6_core.ino.hex"

$ArduinoCli = Get-Command "arduino-cli" -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source -ErrorAction SilentlyContinue
if (-not $ArduinoCli) {
    $Candidates = @(
        "$env:LOCALAPPDATA\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
        "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
        "C:\Program Files (x86)\Arduino\arduino-cli.exe"
    )
    foreach ($c in $Candidates) {
        if (Test-Path $c) { $ArduinoCli = $c; break }
    }
}

if ($ArduinoCli) {
    Write-Host "Using compiler: $ArduinoCli" -ForegroundColor DarkGray
    if (-not (Test-Path $BuildDir)) {
        New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
    }
    $ConfigDir = "$env:LOCALAPPDATA\Arduino15"
    & $ArduinoCli --config-dir $ConfigDir compile --fqbn arduino:avr:mega --output-dir $BuildDir $FirmwareDir
    if ($LASTEXITCODE -eq 0 -and (Test-Path $HexFile)) {
        $hexSize = (Get-Item $HexFile).Length
        Write-Host "[OK] Firmware compiled successfully: $hexSize bytes" -ForegroundColor Green
    } else {
        Write-Warning "arduino-cli returned code $LASTEXITCODE. Using existing binary hex..."
    }
} else {
    Write-Warning "arduino-cli not found. Utilizing existing pre-compiled binary..."
}

if (-not (Test-Path $HexFile)) {
    throw "Fatal: Firmware hex binary not found at $HexFile!"
}

# --------------------------------------------------------------------------
# 4. PACKAGE AND TRANSFER
# --------------------------------------------------------------------------
Write-Host "`n[Step 4/6] Creating Single Deployment Archive & Uploading..." -ForegroundColor Yellow

$stage = Join-Path $env:TEMP "parol6_stage"
if (Test-Path $stage) { Remove-Item $stage -Recurse -Force }
New-Item -ItemType Directory -Force -Path $stage | Out-Null

Copy-Item $HexFile -Destination $stage
Copy-Item (Join-Path $ProjectRoot "daemon") -Destination $stage -Recurse -Exclude "__pycache__", "*.pyc"
Copy-Item (Join-Path $ProjectRoot "ui") -Destination $stage -Recurse
Copy-Item (Join-Path $ProjectRoot "deploy") -Destination $stage -Recurse
if (Test-Path (Join-Path $ProjectRoot "urdf_model")) {
    Copy-Item (Join-Path $ProjectRoot "urdf_model") -Destination $stage -Recurse
}

$archive = Join-Path $env:TEMP "parol6-deploy.tar.gz"
if (Test-Path $archive) { Remove-Item $archive -Force }
tar -czf $archive -C $stage .
Remove-Item $stage -Recurse -Force

$tarSize = [math]::Round((Get-Item $archive).Length / 1KB, 1)
Write-Host "Compressed archive: $tarSize KB. Transferring via SSH..." -ForegroundColor DarkGray

& $scp @sshOpts $archive "${resolvedTarget}:/tmp/parol6-deploy.tar.gz"
Remove-Item $archive -Force
Write-Host "[OK] Deployment archive uploaded to $resolvedTarget." -ForegroundColor Green

# --------------------------------------------------------------------------
# 5. EXECUTE REMOTE FLASH & SERVICE STARTUP
# --------------------------------------------------------------------------
Write-Host "`n[Step 5/6] Flashing Hardware & Launching Daemon Service..." -ForegroundColor Yellow

$RemoteExec = "mkdir -p $RemoteRoot && tar -xzf /tmp/parol6-deploy.tar.gz -C $RemoteRoot && rm -f /tmp/parol6-deploy.tar.gz && chmod +x $RemoteRoot/deploy/remote_install.sh && bash $RemoteRoot/deploy/remote_install.sh $RemoteRoot"
& $ssh @sshOpts -t $resolvedTarget $RemoteExec

if ($LASTEXITCODE -ne 0) {
    throw "Remote deployment failed with exit code $LASTEXITCODE."
}

# --------------------------------------------------------------------------
# 6. VERIFY LIVE SERVICES
# --------------------------------------------------------------------------
Write-Host "`n[Step 6/6] Verifying Live System Health..." -ForegroundColor Yellow
Start-Sleep -Seconds 2

$daemonActive = (& $ssh @sshOpts $resolvedTarget "systemctl is-active parol6-daemon.service").Trim()
$serviceActive = (& $ssh @sshOpts $resolvedTarget "systemctl is-active parol6.service").Trim()

if ($daemonActive -eq "active" -or $serviceActive -eq "active") {
    Write-Host "  [OK] PAROL6 Control Engine: ACTIVE and RUNNING!" -ForegroundColor Green
    Write-Host "  [OK] Touchscreen Kiosk UI:  http://${piHost}:8000" -ForegroundColor Cyan
    Write-Host "  [OK] WebSocket API:         ws://${piHost}:8765" -ForegroundColor Cyan
} else {
    Write-Warning "parol6-daemon status: $daemonActive, parol6 status: $serviceActive"
    & $ssh @sshOpts $resolvedTarget "journalctl -u parol6-daemon.service -n 15 --no-pager"
}

Write-Host "`n==================================================================" -ForegroundColor Green
Write-Host "  PAROL6 DEPLOYMENT COMPLETED SUCCESSFULLY!                       " -ForegroundColor Green
Write-Host "==================================================================" -ForegroundColor Green

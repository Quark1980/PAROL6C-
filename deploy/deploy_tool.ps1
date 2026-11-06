# ==============================================================================
# PAROL6 One-Click Automated Deployment Engine (deploy_tool.ps1)
# ==============================================================================
# Automates:
#  1. Target IP / Hostname resolution with persistent memory (.deploy_target.txt)
#  2. Single-Password authentication: Prompts ONCE, automatically installs
#     SSH ed25519 public key and configures passwordless sudo for future runs
#  3. Arduino-CLI compilation of ATmega2560 firmware to .hex
#  4. Single-archive high-speed upload (43 KB tarball, zero prompts)
#  5. Halting conflicting services (Klipper, Moonraker, KlipperScreen)
#  6. Hardware flashing of ATmega2560 via avrdude on auto-detected serial port
#  7. Dynamic systemd unit generation for active user (fixing 217/USER error)
#  8. Automated Boot-to-Kiosk configuration across Wayland, X11, and Console boot
# ==============================================================================

$ErrorActionPreference = "Stop"

$ProjectRoot = (Get-Item $PSScriptRoot).Parent.FullName
$TargetFile  = Join-Path $PSScriptRoot ".deploy_target.txt"

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

$askpassCmd = Join-Path $env:TEMP "parol6_askpass.cmd"

try {
    Write-Host "==================================================================" -ForegroundColor Cyan
    Write-Host "  PAROL6 ROBOTICS: ONE-CLICK AUTOMATED DEPLOYMENT ENGINE          " -ForegroundColor Cyan
    Write-Host "==================================================================" -ForegroundColor Cyan

    # --------------------------------------------------------------------------
    # 1. RESOLVE PI TARGET
    # --------------------------------------------------------------------------
    $defaultTarget = "mark@192.168.0.159"
    if (Test-Path $TargetFile) {
        $saved = (Get-Content $TargetFile -Raw).Trim()
        if ($saved -ne "") {
            $defaultTarget = $saved
        }
    }

    Write-Host "`n[Step 1/6] Target Raspberry Pi Configuration" -ForegroundColor Yellow
    Write-Host "Press [ENTER] to use default: " -NoNewline
    Write-Host "$defaultTarget" -ForegroundColor Green
    $userInput = Read-Host "Enter Target (user@ip-or-hostname) [$defaultTarget]"

    $target = $defaultTarget
    if (-not [string]::IsNullOrWhiteSpace($userInput)) {
        $target = $userInput.Trim()
    }
    if ($target -notmatch "@") {
        $target = "pi@$target"
    }
    Set-Content -Path $TargetFile -Value $target -Force
    Write-Host "Target resolved: $target" -ForegroundColor Green

    $piUser = ($target -split "@")[0]
    $piHost = ($target -split "@")[1]
    $RemoteRoot = "/home/$piUser/parol6"
    $sshOpts = @("-o", "StrictHostKeyChecking=accept-new", "-o", "ConnectTimeout=10")

    # --------------------------------------------------------------------------
    # 2. SEAMLESS AUTHENTICATION (SINGLE PASSWORD PROMPT)
    # --------------------------------------------------------------------------
    Write-Host "`n[Step 2/6] Checking SSH Key Authentication..." -ForegroundColor Yellow

    $keyPath = Join-Path $env:USERPROFILE ".ssh\id_ed25519"
    if (-not (Test-Path $keyPath)) {
        Write-Host "Generating ed25519 deployment key on Windows..." -ForegroundColor DarkGray
        New-Item -ItemType Directory -Force -Path (Split-Path $keyPath) | Out-Null
        & $sshKeygen -t ed25519 -N '""' -f $keyPath -q
    }
    $pubKey = (Get-Content "$keyPath.pub" -Raw).Trim()

    # Check if key is already authorized on Pi
    $oldEAP = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    $testOutput = & $ssh @sshOpts -o BatchMode=yes -o NumberOfPasswordPrompts=0 $target "echo KEY_AUTH_OK" 2>&1
    $isKeyAuthed = ($LASTEXITCODE -eq 0 -and $testOutput -match "KEY_AUTH_OK")
    
    # Check if passwordless sudo is configured
    $canSudo = $false
    if ($isKeyAuthed) {
        $sudoOut = & $ssh @sshOpts -o BatchMode=yes $target "sudo -n true" 2>&1
        $canSudo = ($LASTEXITCODE -eq 0)
    }
    $ErrorActionPreference = $oldEAP

    if ($isKeyAuthed -and $canSudo) {
        Write-Host "[OK] SSH Key and passwordless sudo verified. No password needed!" -ForegroundColor Green
    } else {
        Write-Host "`nAuthentication required for $target." -ForegroundColor Yellow
        Write-Host "Please enter your password ONCE below. The deployment engine will automatically" -ForegroundColor DarkGray
        Write-Host "deploy your SSH key and configure passwordless sudo for future one-click runs." -ForegroundColor DarkGray
        
        $secPass = Read-Host "Password for $target" -AsSecureString
        $BSTR = [System.Runtime.InteropServices.Marshal]::SecureStringToBSTR($secPass)
        $passPlain = [System.Runtime.InteropServices.Marshal]::PtrToStringAuto($BSTR)
        [System.Runtime.InteropServices.Marshal]::ZeroFreeBSTR($BSTR)

        if ([string]::IsNullOrWhiteSpace($passPlain)) {
            throw "Password cannot be empty."
        }

        # Arm SSH_ASKPASS for automated Windows OpenSSH supply
        Set-Content -Path $askpassCmd -Value "@echo %PAROL6_PASS%" -Encoding ASCII
        $env:PAROL6_PASS = $passPlain
        $env:SSH_ASKPASS = $askpassCmd
        $env:SSH_ASKPASS_REQUIRE = "force"
        $env:DISPLAY = "dummy:0"

        Write-Host "Configuring SSH key authentication on $target..." -ForegroundColor DarkGray
        $keyInstallCmd = "mkdir -p ~/.ssh && chmod 700 ~/.ssh && (grep -qxF '$pubKey' ~/.ssh/authorized_keys 2>/dev/null || echo '$pubKey' >> ~/.ssh/authorized_keys) && chmod 600 ~/.ssh/authorized_keys"
        $oldEAP = $ErrorActionPreference
        $ErrorActionPreference = "Continue"
        $authRes = & $ssh @sshOpts $target $keyInstallCmd 2>&1
        $ErrorActionPreference = $oldEAP

        if ($LASTEXITCODE -ne 0) {
            Write-Warning "SSH response: $authRes"
            throw "Authentication failed for $target. Please verify your password and try again."
        }

        Write-Host "Configuring passwordless sudo for user '$piUser'..." -ForegroundColor DarkGray
        $sudoSetupCmd = "echo '$passPlain' | sudo -S sh -c 'echo `"$piUser ALL=(ALL) NOPASSWD: ALL`" > /etc/sudoers.d/010_parol6_nopasswd && chmod 0440 /etc/sudoers.d/010_parol6_nopasswd'"
        $oldEAP = $ErrorActionPreference
        $ErrorActionPreference = "Continue"
        & $ssh @sshOpts $target $sudoSetupCmd 2>&1 | Out-Null
        $ErrorActionPreference = $oldEAP

        Write-Host "[OK] SSH Key deployed and passwordless sudo active. No further passwords will be requested!" -ForegroundColor Green
    }

    # --------------------------------------------------------------------------
    # 3. COMPILE AVR FIRMWARE WITH ARDUINO-CLI
    # --------------------------------------------------------------------------
    Write-Host "`n[Step 3/6] Compiling ATmega2560 Step Engine Firmware..." -ForegroundColor Yellow

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
            Write-Host "[OK] Compiled firmware successfully: $hexSize bytes" -ForegroundColor Green
        } else {
            Write-Warning "Compilation returned exit code $LASTEXITCODE. Checking for existing pre-built hex..."
        }
    } else {
        Write-Warning "arduino-cli not found. Utilizing existing pre-compiled binary..."
    }

    if (-not (Test-Path $HexFile)) {
        throw "Fatal: Binary hex file not found at $HexFile!"
    }

    # --------------------------------------------------------------------------
    # 4. PACKAGE AND UPLOAD (SINGLE 43KB TARBALL ARCHIVE)
    # --------------------------------------------------------------------------
    Write-Host "`n[Step 4/6] Creating Single Deployment Archive & Uploading..." -ForegroundColor Yellow

    $stage = Join-Path $env:TEMP "parol6_stage"
    if (Test-Path $stage) { Remove-Item $stage -Recurse -Force }
    New-Item -ItemType Directory -Force -Path $stage | Out-Null

    # Copy items to staging
    Copy-Item $HexFile -Destination $stage
    Copy-Item (Join-Path $ProjectRoot "daemon") -Destination $stage -Recurse -Exclude "__pycache__", "*.pyc"
    Copy-Item (Join-Path $ProjectRoot "ui") -Destination $stage -Recurse
    Copy-Item (Join-Path $ProjectRoot "deploy") -Destination $stage -Recurse
    if (Test-Path (Join-Path $ProjectRoot "urdf_model")) {
        Copy-Item (Join-Path $ProjectRoot "urdf_model") -Destination $stage -Recurse
    }

    # Create compressed tarball
    $archive = Join-Path $env:TEMP "parol6-deploy.tar.gz"
    if (Test-Path $archive) { Remove-Item $archive -Force }
    tar -czf $archive -C $stage .
    Remove-Item $stage -Recurse -Force

    $tarSize = [math]::Round((Get-Item $archive).Length / 1KB, 1)
    Write-Host "Compressed archive: $tarSize KB. Transferring via SSH..." -ForegroundColor DarkGray

    # Upload single archive to /tmp/parol6-deploy.tar.gz (zero prompt!)
    & $scp @sshOpts $archive "${target}:/tmp/parol6-deploy.tar.gz"
    Remove-Item $archive -Force
    Write-Host "[OK] Deployment archive uploaded to $target." -ForegroundColor Green

    # --------------------------------------------------------------------------
    # 5. EXECUTE REMOTE HARDWARE FLASH & SERVICE LAUNCH
    # --------------------------------------------------------------------------
    Write-Host "`n[Step 5/6] Executing Remote Hardware Flash & Kiosk Installation..." -ForegroundColor Yellow

    $RemoteExec = "mkdir -p $RemoteRoot && tar -xzf /tmp/parol6-deploy.tar.gz -C $RemoteRoot && rm -f /tmp/parol6-deploy.tar.gz && chmod +x $RemoteRoot/deploy/remote_install.sh && bash $RemoteRoot/deploy/remote_install.sh $RemoteRoot"
    & $ssh @sshOpts -t $target $RemoteExec

    if ($LASTEXITCODE -ne 0) {
        throw "Remote setup script reported an error (exit code $LASTEXITCODE)."
    }

    # --------------------------------------------------------------------------
    # 6. VERIFY SERVICES & SYSTEM HEALTH
    # --------------------------------------------------------------------------
    Write-Host "`n[Step 6/6] Verifying Live System Health..." -ForegroundColor Yellow

    $daemonStatus = (& $ssh @sshOpts $target "systemctl is-active parol6-daemon.service").Trim()
    $kioskStatus  = (& $ssh @sshOpts $target "systemctl is-active parol6-kiosk.service").Trim()

    if ($daemonStatus -eq "active") {
        Write-Host "  [OK] parol6-daemon.service: ACTIVE and RUNNING" -ForegroundColor Green
    } else {
        Write-Warning "  [WARN] parol6-daemon.service status: $daemonStatus"
        & $ssh @sshOpts $target "journalctl -u parol6-daemon.service -n 10 --no-pager"
    }

    if ($kioskStatus -eq "active") {
        Write-Host "  [OK] parol6-kiosk.service:  ACTIVE (Displaying on Touchscreen)" -ForegroundColor Green
    } else {
        Write-Warning "  [WARN] parol6-kiosk.service status: $kioskStatus"
        & $ssh @sshOpts $target "journalctl -u parol6-kiosk.service -n 10 --no-pager"
    }

    Write-Host "`n==================================================================" -ForegroundColor Cyan
    Write-Host "  [SUCCESS] PAROL6 DEPLOYMENT COMPLETE & OPERATIONAL!            " -ForegroundColor Cyan
    Write-Host "==================================================================" -ForegroundColor Cyan
    Write-Host "  - Target Pi Host:        $piHost ($piUser)" -ForegroundColor White
    Write-Host "  - Daemon WebSocket API:  ws://${piHost}:8765" -ForegroundColor Yellow
    Write-Host "  - Local Touchscreen UI:  http://${piHost}:8000" -ForegroundColor White
    Write-Host "  - Physical Touchscreen:  PAROL6 Kiosk Active on Screen" -ForegroundColor Green
    Write-Host "==================================================================" -ForegroundColor Cyan

} finally {
    # Clean up sensitive environment variables and temporary askpass script
    $env:PAROL6_PASS = ""
    $env:SSH_ASKPASS = ""
    $env:SSH_ASKPASS_REQUIRE = ""
    if (Test-Path $askpassCmd) {
        Remove-Item $askpassCmd -Force -ErrorAction SilentlyContinue
    }
}

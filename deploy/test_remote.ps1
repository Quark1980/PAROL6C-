<#
.SYNOPSIS
    PAROL6 Remote Hardware-In-The-Loop (HIL) Execution Script
.DESCRIPTION
    Syncs daemon files to Raspberry Pi 5, restarts service, executes
    daemon/tools/run_hardware_tests.py over SSH, and inspects results.
#>

param(
    [string]$TargetHost = "192.168.0.159",
    [string]$TargetUser = "mark",
    [string]$RemoteDir = "/home/mark/parol6",
    [switch]$SkipSync
)

$ErrorActionPreference = "Stop"

Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host "PAROL6 AUTOMATED REMOTE HARDWARE-IN-THE-LOOP (HIL) TEST SUITE" -ForegroundColor Cyan
Write-Host "Target: $TargetUser@$TargetHost" -ForegroundColor Cyan
Write-Host "=================================================================" -ForegroundColor Cyan

if (-not $SkipSync) {
    Write-Host "`n[*] Syncing latest daemon files to Raspberry Pi 5..." -ForegroundColor Yellow
    
    # Ensure remote directories exist
    ssh -o ConnectTimeout=5 "$TargetUser@$TargetHost" "mkdir -p $RemoteDir/daemon/tools"
    
    # Sync python files
    scp "c:\Users\mseij\Desktop\PAROLC_Firmware\daemon\kinematics.py" "$TargetUser@${TargetHost}:$RemoteDir/daemon/"
    scp "c:\Users\mseij\Desktop\PAROLC_Firmware\daemon\trajectory_planner.py" "$TargetUser@${TargetHost}:$RemoteDir/daemon/"
    scp "c:\Users\mseij\Desktop\PAROLC_Firmware\daemon\robot_engine.py" "$TargetUser@${TargetHost}:$RemoteDir/daemon/"
    scp "c:\Users\mseij\Desktop\PAROLC_Firmware\daemon\tools\run_hardware_tests.py" "$TargetUser@${TargetHost}:$RemoteDir/daemon/tools/"
    
    Write-Host "[+] Files synced successfully." -ForegroundColor Green
    
    Write-Host "[*] Restarting parol6-daemon.service on Raspberry Pi 5..." -ForegroundColor Yellow
    ssh "$TargetUser@$TargetHost" "sudo systemctl restart parol6-daemon.service"
    Start-Sleep -Seconds 2
}

Write-Host "`n[*] Executing run_hardware_tests.py over SSH..." -ForegroundColor Yellow
$remoteCmd = "python3 $RemoteDir/daemon/tools/run_hardware_tests.py --all"

$hilExitCode = 0
try {
    ssh -t "$TargetUser@$TargetHost" $remoteCmd
    $hilExitCode = $LASTEXITCODE
} catch {
    $hilExitCode = 1
}

Write-Host "`n[*] Fetching remote test report from /tmp/hil_test_results.json..." -ForegroundColor Yellow
$localReport = "$PSScriptRoot\..\docs\hil_test_results.json"
try {
    scp "$TargetUser@${TargetHost}:/tmp/hil_test_results.json" $localReport
    if (Test-Path $localReport) {
        $jsonContent = Get-Content $localReport -Raw | ConvertFrom-Json
        Write-Host "`nStructured HIL Summary:" -ForegroundColor Cyan
        Write-Host "  All Passed: $($jsonContent.all_passed)" -ForegroundColor $(if ($jsonContent.all_passed) { "Green" } else { "Red" })
        foreach ($prop in $jsonContent.tests.PSObject.Properties) {
            $tName = $prop.Name
            $tPassed = $prop.Value.passed
            $tStatus = if ($tPassed) { "[PASS]" } else { "[FAIL]" }
            $color = if ($tPassed) { "Green" } else { "Red" }
            Write-Host "  $($tName.ToUpper()): $tStatus" -ForegroundColor $color
        }
    }
} catch {
    Write-Host "[!] Could not fetch or parse JSON report: $_" -ForegroundColor Yellow
}

if ($hilExitCode -eq 0) {
    Write-Host "`n[PASS] ALL HIL BENCHMARK ASSERTIONS SATISFIED." -ForegroundColor Green
    exit 0
} else {
    Write-Host "`n[FAIL] HIL TEST SUITE RETURNED FAILURE (Exit Code $hilExitCode)." -ForegroundColor Red
    exit 1
}

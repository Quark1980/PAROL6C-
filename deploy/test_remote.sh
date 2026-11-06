#!/usr/bin/env bash
# PAROL6 Remote Hardware-In-The-Loop (HIL) Execution Script

set -euo pipefail

TARGET_HOST="${1:-192.168.0.159}"
TARGET_USER="${2:-mark}"
REMOTE_DIR="${3:-/home/mark/parol6}"

echo "================================================================="
echo "PAROL6 AUTOMATED REMOTE HARDWARE-IN-THE-LOOP (HIL) TEST SUITE"
echo "Target: ${TARGET_USER}@${TARGET_HOST}"
echo "================================================================="

echo "[*] Syncing latest daemon files to Raspberry Pi 5..."
ssh -o ConnectTimeout=5 "${TARGET_USER}@${TARGET_HOST}" "mkdir -p ${REMOTE_DIR}/daemon/tools"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"

scp "${PARENT_DIR}/daemon/kinematics.py" "${TARGET_USER}@${TARGET_HOST}:${REMOTE_DIR}/daemon/"
scp "${PARENT_DIR}/daemon/trajectory_planner.py" "${TARGET_USER}@${TARGET_HOST}:${REMOTE_DIR}/daemon/"
scp "${PARENT_DIR}/daemon/robot_engine.py" "${TARGET_USER}@${TARGET_HOST}:${REMOTE_DIR}/daemon/"
scp "${PARENT_DIR}/daemon/tools/run_hardware_tests.py" "${TARGET_USER}@${TARGET_HOST}:${REMOTE_DIR}/daemon/tools/"

echo "[*] Restarting parol6-daemon.service on Raspberry Pi 5..."
ssh "${TARGET_USER}@${TARGET_HOST}" "sudo systemctl restart parol6-daemon.service"
sleep 2

echo "[*] Executing run_hardware_tests.py over SSH..."
ssh -t "${TARGET_USER}@${TARGET_HOST}" "python3 ${REMOTE_DIR}/daemon/tools/run_hardware_tests.py --all"

echo "[*] Fetching remote test report from /tmp/hil_test_results.json..."
mkdir -p "${PARENT_DIR}/docs"
scp "${TARGET_USER}@${TARGET_HOST}:/tmp/hil_test_results.json" "${PARENT_DIR}/docs/hil_test_results.json" || true

echo "[+] Remote HIL Test Execution Complete."

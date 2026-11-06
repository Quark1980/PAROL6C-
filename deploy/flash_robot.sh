#!/usr/bin/env bash
# ==============================================================================
# PAROL6 Cross-Deployment Script (Bash for Linux / macOS / Raspberry Pi local)
# ==============================================================================

set -euo pipefail

PI_HOST="${1:-localhost}"
PI_USER="${2:-pi}"
SERIAL_PORT="${3:-/dev/ttyACM0}"

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HEX_FILE="${WORKSPACE}/firmware/build/parol6_core.ino.hex"

echo "============================================================"
echo " 🤖 PAROL6 ROBOT CONTROL DEPLOYMENT PIPELINE"
echo " Host: ${PI_HOST} | User: ${PI_USER} | Port: ${SERIAL_PORT}"
echo "============================================================"

# If running directly on the Raspberry Pi
if [ "${PI_HOST}" = "localhost" ] || [ "${PI_HOST}" = "127.0.0.1" ]; then
    echo "[1/1] Running automated setup & kiosk installer..."
    bash "${WORKSPACE}/deploy/remote_install.sh" "${WORKSPACE}" "${SERIAL_PORT}"
    echo "✓ Deployment and boot-to-kiosk setup complete locally on Raspberry Pi!"
    exit 0
fi

# If deploying from remote Linux / macOS workstation to Raspberry Pi
REMOTE_DIR="/home/${PI_USER}/parol6"

echo "[1/3] Preparing remote directories on ${PI_HOST}..."
ssh "${PI_USER}@${PI_HOST}" "mkdir -p ${REMOTE_DIR}/daemon ${REMOTE_DIR}/ui ${REMOTE_DIR}/deploy"

echo "[2/3] Transferring binary, daemon and kiosk assets..."
scp "${HEX_FILE}" "${PI_USER}@${PI_HOST}:${REMOTE_DIR}/parol6_core.ino.hex"
scp -r "${WORKSPACE}/daemon/"* "${PI_USER}@${PI_HOST}:${REMOTE_DIR}/daemon/"
scp -r "${WORKSPACE}/ui/"* "${PI_USER}@${PI_HOST}:${REMOTE_DIR}/ui/"
scp -r "${WORKSPACE}/deploy/"* "${PI_USER}@${PI_HOST}:${REMOTE_DIR}/deploy/"
scp "${WORKSPACE}/deploy/parol6-daemon.service" "${PI_USER}@${PI_HOST}:${REMOTE_DIR}/"

echo "[3/3] Executing remote installer, hardware flash & kiosk autostart configuration..."
ssh -t "${PI_USER}@${PI_HOST}" "bash ${REMOTE_DIR}/deploy/remote_install.sh ${REMOTE_DIR} ${SERIAL_PORT}"

echo "✓ Deployment and boot-to-kiosk setup successful to ${PI_HOST}!"

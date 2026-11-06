#!/usr/bin/env bash
# ==============================================================================
# PAROL6 Cross-Platform Deployment Script (deploy_robot.sh)
# ==============================================================================
# 1. Compiles ATmega2560 firmware using arduino-cli
# 2. Stops conflicting services (klipper, moonraker, klipperscreen)
# 3. Flashes ATmega2560 on RAMPS 1.4 via avrdude
# 4. Installs and starts parol6.service systemd unit
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/firmware/build"
FIRMWARE_DIR="$PROJECT_ROOT/firmware/parol6_core"
HEX_FILE="$BUILD_DIR/parol6_core.ino.hex"

echo "=================================================================="
echo "  PAROL6 ROBOTICS: CROSS-PLATFORM DEPLOYMENT SCRIPT               "
echo "=================================================================="

# 1. Compile firmware if arduino-cli is installed
if command -v arduino-cli &> /dev/null; then
    echo ">>> Compiling ATmega2560 firmware with arduino-cli..."
    mkdir -p "$BUILD_DIR"
    arduino-cli compile --fqbn arduino:avr:mega --output-dir "$BUILD_DIR" "$FIRMWARE_DIR"
    echo "[OK] Firmware compiled successfully."
else
    echo ">>> arduino-cli not found, verifying pre-compiled binary..."
fi

if [ ! -f "$HEX_FILE" ]; then
    echo "[ERROR] Firmware binary not found at $HEX_FILE!"
    exit 1
fi

# 2. Halt conflicting services
echo ">>> Halting conflicting services..."
sudo systemctl stop klipper moonraker KlipperScreen parol6-daemon parol6 2>/dev/null || true

# 3. Locate serial port
PORT="/dev/ttyACM0"
if [ ! -e "$PORT" ]; then
    DETECTED=$(ls /dev/serial/by-id/* 2>/dev/null | head -n 1 || true)
    if [ -n "$DETECTED" ]; then
        PORT="$DETECTED"
    fi
fi
echo ">>> Using serial port: $PORT"

# 4. Flash hardware via avrdude
echo ">>> Flashing ATmega2560 via avrdude (-c wiring -p m2560 -b 115200)..."
sudo avrdude -c wiring -p m2560 -P "$PORT" -b 115200 -D -U flash:w:"$HEX_FILE":i
echo "[OK] Flash verified successfully."

# 5. Configure systemd background service
echo ">>> Configuring systemd unit parol6.service..."
CURRENT_USER=$(id -un)
CURRENT_DIR="$PROJECT_ROOT"

cat << EOF | sudo tee /etc/systemd/system/parol6.service > /dev/null
[Unit]
Description=PAROL6 6-DOF Robot Trajectory & Kinematics Control Engine
After=network.target local-fs.target
Wants=network.target

[Service]
Type=simple
User=$CURRENT_USER
Group=$CURRENT_USER
WorkingDirectory=$CURRENT_DIR
ExecStart=/usr/bin/python3 $CURRENT_DIR/daemon/robot_engine.py
Restart=always
RestartSec=3
StandardOutput=journal
StandardError=journal
LimitRTPRIO=99
LimitMEMLOCK=infinity

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable parol6.service
sudo systemctl restart parol6.service

echo "[OK] parol6.service started."
sudo systemctl is-active parol6.service
echo "Deployment complete!"

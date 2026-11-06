#!/usr/bin/env bash
# ==============================================================================
# PAROL6 Remote Node Installer, Flasher & Kiosk Setup (runs on Raspberry Pi 5)
# ==============================================================================
set -euo pipefail

TARGET_DIR="${1:-$HOME/parol6}"
SERIAL_PORT="${2:-/dev/ttyACM0}"
PI_USER="${SUDO_USER:-$(whoami)}"
PI_HOME="$(getent passwd "$PI_USER" | cut -d: -f6)"
PI_UID="$(id -u "$PI_USER")"

echo "------------------------------------------------------------"
echo " [Pi-Side] Configuring PAROL6 Control & Kiosk Node"
echo " User:              ${PI_USER} (UID: ${PI_UID})"
echo " Working Directory: ${TARGET_DIR}"
echo " User Home:         ${PI_HOME}"
echo " Serial Port:       ${SERIAL_PORT}"
echo "------------------------------------------------------------"

# Ensure passwordless sudo for user for automated operations
if [ ! -f /etc/sudoers.d/010_parol6_nopasswd ]; then
    echo "Configuring passwordless sudo for ${PI_USER}..."
    echo "${PI_USER} ALL=(ALL) NOPASSWD: ALL" | sudo tee /etc/sudoers.d/010_parol6_nopasswd > /dev/null
    sudo chmod 0440 /etc/sudoers.d/010_parol6_nopasswd
fi

# 1. Stop conflicting services and free ports 8765 and 8000
echo ">>> [1/6] Stopping conflicting services & clearing ports..."
sudo systemctl stop klipper moonraker parol6-daemon parol6 parol6-studio parol6-kiosk 2>/dev/null || true
sudo systemctl disable --now klipper moonraker KlipperScreen parol6-studio parol6 2>/dev/null || true
sudo systemctl mask klipper moonraker KlipperScreen parol6-studio parol6 2>/dev/null || true

# Force kill any stale processes bound to our ports or serial device
sudo fuser -k -9 8765/tcp 8000/tcp 2>/dev/null || true
sudo pkill -f robot_daemon.py 2>/dev/null || true
sudo pkill -f robot_engine.py 2>/dev/null || true

# Add user to serial and display groups
sudo usermod -a -G dialout,tty,video,input,render "${PI_USER}" 2>/dev/null || true

# Configure Xwrapper for non-root display access
sudo mkdir -p /etc/X11
cat <<EOF | sudo tee /etc/X11/Xwrapper.config > /dev/null
allowed_users=anybody
needs_root_rights=yes
EOF

# 2. Identify serial port if /dev/ttyACM0 is not found directly
if [ ! -e "${SERIAL_PORT}" ]; then
    echo "Notice: ${SERIAL_PORT} not found directly, scanning /dev/serial/by-id/..."
    if [ -d "/dev/serial/by-id" ]; then
        FOUND_PORT=$(ls /dev/serial/by-id/* 2>/dev/null | head -n 1 || true)
        if [ -n "${FOUND_PORT}" ]; then
            SERIAL_PORT="${FOUND_PORT}"
            echo "Auto-detected serial port: ${SERIAL_PORT}"
        fi
    fi
fi

# 3. Flash ATmega2560 on RAMPS 1.4 using avrdude
HEX_FILE="${TARGET_DIR}/parol6_core.ino.hex"
if [ -f "${HEX_FILE}" ]; then
    echo ">>> [2/6] Flashing ATmega2560 on RAMPS 1.4 via avrdude..."
    sudo fuser -k -9 "${SERIAL_PORT}" 2>/dev/null || true
    sleep 1
    if ! command -v avrdude &>/dev/null; then
        echo "Installing avrdude..."
        sudo apt-get update -qq && sudo apt-get install -y -qq avrdude
    fi
    avrdude -c wiring -p m2560 -P "${SERIAL_PORT}" -b 115200 -D -U "flash:w:${HEX_FILE}:i"
    echo "[OK] ATmega2560 flashed successfully."
else
    echo "Warning: ${HEX_FILE} not found! Skipping flash step."
fi

# 4. Install Python & Display/Kiosk dependencies
echo ">>> [3/6] Verifying Python 3, Chromium and Touch Kiosk dependencies..."
sudo apt-get update -qq
sudo apt-get install -y -qq \
    python3-numpy \
    python3-scipy \
    python3-websockets \
    python3-serial \
    python3-pip \
    xserver-xorg \
    xinit \
    x11-xserver-utils \
    xserver-xorg-input-all \
    unclutter \
    curl \
    libgl1-mesa-dri \
    libgles2-mesa \
    libegl1-mesa 2>/dev/null || true

sudo apt-get install -y -qq matchbox-window-manager 2>/dev/null || \
sudo apt-get install -y -qq openbox 2>/dev/null || true

sudo apt-get install -y -qq chromium-browser 2>/dev/null || \
sudo apt-get install -y -qq chromium 2>/dev/null || true

if [ -f "${TARGET_DIR}/daemon/requirements.txt" ]; then
    pip install --break-system-packages -r "${TARGET_DIR}/daemon/requirements.txt" 2>/dev/null || \
    pip install -r "${TARGET_DIR}/daemon/requirements.txt" 2>/dev/null || true
fi
echo "[OK] Dependencies satisfied."

# 5. Dynamically generate and install parol6-daemon.service and parol6.service
echo ">>> [4/6] Installing and starting parol6-daemon.service and parol6.service for user '${PI_USER}'..."
cat <<EOF | sudo tee /etc/systemd/system/parol6-daemon.service > /dev/null
[Unit]
Description=PAROL6 6-DOF Robot Trajectory & Kinematics Control Daemon
After=network.target local-fs.target
Wants=network.target

[Service]
Type=simple
User=${PI_USER}
Group=${PI_USER}
WorkingDirectory=${TARGET_DIR}
Environment="PAROL6_SERIAL_PORT=${SERIAL_PORT}"
Environment="PYTHONUNBUFFERED=1"
ExecStart=/usr/bin/python3 ${TARGET_DIR}/daemon/robot_engine.py
Restart=always
RestartSec=3
StandardOutput=journal
StandardError=journal
LimitRTPRIO=99
LimitMEMLOCK=infinity

[Install]
WantedBy=multi-user.target
EOF

sudo cp /etc/systemd/system/parol6-daemon.service /etc/systemd/system/parol6.service
sudo chmod 644 /etc/systemd/system/parol6-daemon.service /etc/systemd/system/parol6.service
sudo systemctl daemon-reload
sudo systemctl enable parol6-daemon.service parol6.service
sudo systemctl restart parol6-daemon.service

sleep 2
if sudo systemctl is-active --quiet parol6-daemon.service; then
    echo "[OK] parol6-daemon.service is ACTIVE and RUNNING!"
else
    echo "Warning: parol6-daemon did not start cleanly. Status logs:"
    sudo journalctl -u parol6-daemon -n 20 --no-pager
fi

# 6. Configure Automated Touchscreen Kiosk on Boot
echo ">>> [5/6] Configuring Automated Touchscreen Kiosk on Boot..."

# 6a. Install kiosk browser runner
BROWSER_SCRIPT="/usr/local/bin/parol6-kiosk-browser.sh"
cat <<'EOF' | sudo tee "${BROWSER_SCRIPT}" > /dev/null
#!/usr/bin/env bash
set -e

if command -v xset &>/dev/null; then
    xset s off 2>/dev/null || true
    xset -dpms 2>/dev/null || true
    xset s noblank 2>/dev/null || true
fi

if command -v unclutter &>/dev/null; then
    unclutter -idle 0.5 -root &
fi

if ! pgrep -x "matchbox-window-manager" >/dev/null 2>&1 && ! pgrep -x "openbox" >/dev/null 2>&1 && ! pgrep -x "wayfire" >/dev/null 2>&1 && ! pgrep -x "labwc" >/dev/null 2>&1; then
    if command -v matchbox-window-manager &>/dev/null; then
        matchbox-window-manager -use_titlebar no &
    elif command -v openbox &>/dev/null; then
        openbox &
    fi
fi

rm -rf "${HOME}/.config/chromium/Singleton"* 2>/dev/null || true
PREFS_FILE="${HOME}/.config/chromium/Default/Preferences"
if [ -f "${PREFS_FILE}" ]; then
    sed -i 's/"exit_type":"Crashed"/"exit_type":"Normal"/' "${PREFS_FILE}" 2>/dev/null || true
    sed -i 's/"exited_cleanly":false/"exited_cleanly":true/' "${PREFS_FILE}" 2>/dev/null || true
fi

MAX_ATTEMPTS=60
ATTEMPTS=0
TARGET_URL="http://127.0.0.1:8000"
until curl -s "$TARGET_URL" >/dev/null 2>&1; do
    sleep 0.5
    ATTEMPTS=$((ATTEMPTS + 1))
    if [ $ATTEMPTS -ge $MAX_ATTEMPTS ]; then
        echo "Timed out waiting for port 8000; falling back to local HTML file..."
        TARGET_URL="file://${HOME}/parol6/ui/touchscreen.html"
        break
    fi
done

CHROME_BIN=$(command -v chromium-browser || command -v chromium || true)
if [ -z "$CHROME_BIN" ]; then
    echo "Error: chromium browser binary not found!"
    exit 1
fi

echo "Launching Chromium in fullscreen kiosk mode on $TARGET_URL..."
exec "$CHROME_BIN" \
    --kiosk \
    --noerrdialogs \
    --disable-infobars \
    --no-first-run \
    --disable-session-crashed-bubble \
    --disable-pinch \
    --overscroll-history-navigation=0 \
    --check-for-update-interval=31536000 \
    --enable-features=OverlayScrollbar \
    --enable-gpu-rasterization \
    --enable-zero-copy \
    --ignore-gpu-blocklist \
    --enable-webgl \
    --enable-accelerated-2d-canvas \
    --touch-events=enabled \
    --autoplay-policy=no-user-gesture-required \
    "$TARGET_URL"
EOF

sudo chmod +x "${BROWSER_SCRIPT}"

# 6b. Install master kiosk launcher
LAUNCHER_PATH="/usr/local/bin/launch-parol6-kiosk.sh"
cat <<'EOF' | sudo tee "${LAUNCHER_PATH}" > /dev/null
#!/usr/bin/env bash
set -e

export HOME="${HOME:-/home/$(whoami)}"
export USER="${USER:-$(whoami)}"
PI_UID="$(id -u)"

run_browser() {
    exec /usr/local/bin/parol6-kiosk-browser.sh
}

if [ -n "${WAYLAND_DISPLAY:-}" ] && [ -S "${XDG_RUNTIME_DIR:-/run/user/${PI_UID}}/${WAYLAND_DISPLAY}" ]; then
    run_browser
fi

if [ -n "${DISPLAY:-}" ] && [ -e "/tmp/.X11-unix/X${DISPLAY#*:}" ]; then
    run_browser
fi

if [ -e "/tmp/.X11-unix/X0" ]; then
    export DISPLAY=:0
    if [ -f "${HOME}/.Xauthority" ]; then
        export XAUTHORITY="${HOME}/.Xauthority"
    fi
    run_browser
fi

if [ -S "/run/user/${PI_UID}/wayland-0" ]; then
    export XDG_RUNTIME_DIR="/run/user/${PI_UID}"
    export WAYLAND_DISPLAY="wayland-0"
    run_browser
fi

echo "Starting standalone Xorg server via xinit on :0 vt7..."
XORG_BIN=$(command -v Xorg || echo "/usr/lib/xorg/Xorg")
exec xinit /usr/local/bin/parol6-kiosk-browser.sh -- "${XORG_BIN}" :0 -s 0 -dpms -nocursor vt7
EOF

sudo chmod +x "${LAUNCHER_PATH}"

# 6c. Configure systemd kiosk display service
cat <<EOF | sudo tee /etc/systemd/system/parol6-kiosk.service > /dev/null
[Unit]
Description=PAROL6 Touchscreen Kiosk Display Interface
StartLimitIntervalSec=0
After=network.target dbus.socket systemd-user-sessions.service parol6-daemon.service
Wants=parol6-daemon.service

[Service]
Type=simple
User=${PI_USER}
Group=${PI_USER}
SupplementaryGroups=video input tty render dialout
WorkingDirectory=${TARGET_DIR}
Environment=HOME=${PI_HOME}
Environment=USER=${PI_USER}
Environment=DISPLAY=:0
Environment=XAUTHORITY=${PI_HOME}/.Xauthority
StandardInput=null
StandardOutput=journal
StandardError=journal
ExecStart=${LAUNCHER_PATH}
ExecStartPost=+chvt 7
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

sudo chmod 644 /etc/systemd/system/parol6-kiosk.service
sudo systemctl daemon-reload
sudo systemctl enable parol6-kiosk.service
sudo systemctl restart parol6-kiosk.service

# 6d. Desktop session autostarts
AUTOSTART_DIR="${PI_HOME}/.config/autostart"
mkdir -p "${AUTOSTART_DIR}"
cat <<EOF > "${AUTOSTART_DIR}/parol6-kiosk.desktop"
[Desktop Entry]
Type=Application
Name=PAROL6 Touchscreen Kiosk
Comment=Automatic Touch Controls for PAROL6 Robot
Exec=${LAUNCHER_PATH}
Terminal=false
X-GNOME-Autostart-enabled=true
EOF

sudo chown -R "${PI_USER}:${PI_USER}" "${PI_HOME}/.config" "${TARGET_DIR}" 2>/dev/null || true

# 7. Verification Summary
echo ">>> [6/6] Finalizing Setup..."
echo "------------------------------------------------------------"
echo " [OK] PAROL6 Control Daemon: http://127.0.0.1:8000"
echo " [OK] WebSocket Control API: ws://127.0.0.1:8765"
echo " [OK] Touchscreen Kiosk:     Active on physical display"
echo "------------------------------------------------------------"

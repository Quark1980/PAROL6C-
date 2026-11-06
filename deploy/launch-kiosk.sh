#!/usr/bin/env bash
# ==============================================================================
# PAROL6 Touchscreen Kiosk Master Launcher (Raspberry Pi 5)
# Supports:
#  - Standalone Xorg via xinit (MainsailOS Lite / Debian Lite / Headless boot)
#  - Active Desktop Session (X11 / LightDM / PIXEL)
#  - Modern Wayland Session (Labwc / Wayfire / Sway)
# ==============================================================================
set -e

export HOME="${HOME:-/home/$(whoami)}"
export USER="${USER:-$(whoami)}"
PI_UID="$(id -u)"

echo "PAROL6 Kiosk Launcher starting for user ${USER} (UID: ${PI_UID})..."

# Function: run the browser stack inside an active display
run_browser() {
    exec /usr/local/bin/parol6-kiosk-browser.sh
}

# 1. If WAYLAND_DISPLAY is active in current environment
if [ -n "${WAYLAND_DISPLAY:-}" ] && [ -S "${XDG_RUNTIME_DIR:-/run/user/${PI_UID}}/${WAYLAND_DISPLAY}" ]; then
    echo "Active Wayland compositor detected: ${WAYLAND_DISPLAY}. Launching kiosk..."
    run_browser
fi

# 2. If DISPLAY is active and X11 socket exists
if [ -n "${DISPLAY:-}" ] && [ -e "/tmp/.X11-unix/X${DISPLAY#*:}" ]; then
    echo "Active X11 display detected: ${DISPLAY}. Launching kiosk..."
    run_browser
fi

# 3. If an X server is running on :0 (e.g. desktop login manager / LightDM)
if [ -e "/tmp/.X11-unix/X0" ]; then
    echo "Existing X server found on :0. Connecting..."
    export DISPLAY=:0
    if [ -f "${HOME}/.Xauthority" ]; then
        export XAUTHORITY="${HOME}/.Xauthority"
    fi
    run_browser
fi

# 4. If Wayland socket wayland-0 exists in runtime directory
if [ -S "/run/user/${PI_UID}/wayland-0" ]; then
    echo "Wayland compositor socket detected in /run/user/${PI_UID}/wayland-0..."
    export XDG_RUNTIME_DIR="/run/user/${PI_UID}"
    export WAYLAND_DISPLAY="wayland-0"
    run_browser
fi

# 5. Otherwise: Standalone Console Boot (Standard MainsailOS / Lite / KlipperScreen)
# Spawn isolated X server on the physical display via xinit
echo "No running display server detected. Starting standalone Xorg server via xinit on :0 vt7..."
XORG_BIN=$(command -v Xorg || echo "/usr/lib/xorg/Xorg")
exec xinit /usr/local/bin/parol6-kiosk-browser.sh -- "${XORG_BIN}" :0 -s 0 -dpms -nocursor vt7

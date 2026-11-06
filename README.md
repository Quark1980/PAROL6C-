# PAROL6 6-DOF Robotic Control Ecosystem

> **Production-Ready, Deterministic Real-Time Robotic Motion Engine & Kinematics Daemon**  
> Direct ATmega2560 + RAMPS 1.4 AVR Register Stepping Core | Raspberry Pi 5 Host Trajectory Engine | Chromium Touchscreen Kiosk

---

## 1. System Overview

This ecosystem replaces the legacy Klipper-based control suite with a deterministic, hardware-scheduled stepping engine running directly on an 8-bit **Arduino Mega 2560 + RAMPS 1.4**. All 6-DOF kinematics (analytical & Levenberg-Marquardt IK), trajectory blending, touchscreen serving, network communications, and persistent homing calibration are strictly offloaded to a **Raspberry Pi 5**.

### Key Architectural Highlights:
- **NO G-Code Parsing**: Motion commands are streamed directly as high-frequency joint targets (`SERVO_JOINTS`) or discrete quadratic step chunks (`'Q'`).
- **NO Delay Loops in Motion**: Pure event-driven stepping scheduled via 16 MHz Timer 1 virtualized 32-bit hardware clock ($62.5\text{ ns}$ resolution). When the robot is stationary, the step interrupt rate is **0 Hz**.
- **Optimized Multi-Stage Parallel Homing**:
  - **Stage 1 (Parallel)**: Axis **J1, J2, and J3** home concurrently (`[J1+J2+J3]`) with independent soft-start acceleration, backoff, and 5-sample debounced precision latching.
  - **Stage 2 (Forearm)**: **J4** seeks endstop, precision latches, and retreats to $-260.0^\circ$.
  - **Stage 3 (Wrist Pass 1)**: **J6** seeks endstop, precision latches, and retreats to $-4.0^\circ$.
  - **Stage 4 (Wrist Pitch)**: **J5** pre-moves J6 by $-10.0^\circ$, seeks endstop, precision latches, retreats to $+120.0^\circ$, and post-moves J6 by $-5.0^\circ$.
  - **Stage 5 (Wrist Pass 2)**: **J6** executes a dedicated second homing pass to completely eliminate the $-15.0^\circ$ displacement incurred during J5's clearance maneuvers.
  - **Stage 6 (CAD Zero Reference)**: Physical step positions are locked directly to the PAROL6 CAD zero definition ($J_1=0^\circ, J_2=-90^\circ, J_3=180^\circ, J_4=0^\circ, J_5=0^\circ, J_6=180^\circ$) and the safety interlock is unlocked (`homed_mask == 0x3F`).
- **Hardware Polarity Ground Truth**:
  - `dir_invert_mask`: `0x03` (`0b00000011` — only J1 `!PF1` and J2 `!PF7` inverted, matching `current printer.cfg`).
  - `endstop_invert_mask`: `0x00` (all 6 limit switches active-high pulled-up `^PE5`, `^PE4`, `^PJ1`, `^PJ0`, `^PD3`, `^PD2`).
  - Automated polarity sync on serial connect and `RESET_INVERT_MASKS` WebSocket API.
- **TI DRV8825 Step Pulse Timing**: Direct AVR port register toggles guarantee step pulse hold $\ge 2.5\ \mu\text{s}$ via inline assembly delays (`__builtin_avr_delay_cycles(42)`), meeting the DRV8825 $t_{\text{WH(STEP)}}$ specification across all 6 axes.
- **Hardware Safety Interlock**: Both the MCU and host daemon strictly reject all joint moves, jogging, and streaming packets until the hardware homing sequence has completed successfully (`homed_mask == 0x3F`).
- **Persistent Homing Offset Trims**: Switch position adjustments and mechanical hysteresis are calibrated dynamically on the Pi via `offsets.json` and the touchscreen UI without re-flashing firmware.
- **Zero-Latency Touchscreen E-Stop**: The on-screen Emergency Stop button is wired to `pointerdown` for immediate network execution and MCU coil de-energization.

---

## 2. Distributed Architecture

```
[Windows Control Suite / Remote Client]
          │
          │ WebSocket (ws://<pi_ip>:8765)
          │ SERVO_JOINTS (20-50 Hz) | STREAM_TRAJECTORY | 20 Hz Telemetry
          ▼
┌─────────────────────────────────────────────────────────────┐
│ Raspberry Pi 5 (Host Daemon)                                │
│  ├── robot_engine.py: Python 3 Trajectory & Kinematics Core │
│  │    ├── Levenberg-Marquardt (LM) Numerical IK             │
│  │    ├── 100 Hz Trapezoidal Trajectory & Velocity Blending │
│  │    ├── Look-Ahead Step Chunk Queue (80-120 ms buffer)    │
│  │    ├── Master Clock Drift Estimator                      │
│  │    └── Persistent Calibration Manager (offsets.json)     │
│  ├── Local Touchscreen Kiosk UI (http://127.0.0.1:8000)     │
│  │    ├── Chromium Kiosk Mode (parol6-kiosk.service)        │
│  │    ├── Zero-latency E-Stop (pointerdown)                 │
│  │    └── Homing Calibration Sub-Panel (±0.1° / ±1.0°)      │
│  └── Serial Bridge Worker Thread (Decoupled Non-Blocking)   │
└──────────────────────────────┬──────────────────────────────┘
                               │ High-Speed USB-UART @ 250,000 baud
                               │ Binary Step Chunk Protocol ('Q' / 'P')
                               ▼
┌─────────────────────────────────────────────────────────────┐
│ Arduino Mega 2560 + RAMPS 1.4 (Pure Stepper Execution Core) │
│  ├── 16 MHz 16-bit Timer 1 virtualized to 32-bit clock      │
│  ├── Dynamic OCR1A step scheduling (0 Hz when stationary)   │
│  ├── Direct Port Register manipulation (PORTx / DDRx)       │
│  ├── DRV8825 >= 2.5 us pulse width guarantee                │
│  ├── Soft-Start Homing State Machine & Debounced Latch      │
│  └── Timer 3 Hardware PWM RC Servo Driver (D11 / D4)        │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. Physical Hardware Pin Mappings

Directly maps to the RAMPS 1.4 shield on ATmega2560:

| Axis | Function | Step Pin | Dir Pin | Enable Pin | Limit Switch Pin | Gear Ratio | Steps / Degree | Soft Limits |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **J1** | Base Yaw | A0 (`PF0`) | A1 (`PF1`) [Inv] | D38 (`PD7`) | X-min (`PE5`) | 10:1 (GT2 160:16) | **88.888889** | $-175^\circ \dots +175^\circ$ |
| **J2** | Shoulder | A6 (`PF6`) | A7 (`PF7`) [Inv] | A2 (`PF2`) | X-max (`PE4`) | 50:1 Planetary | **444.444444** | $-150^\circ \dots +50^\circ$ |
| **J3** | Elbow | D46 (`PL3`) | D48 (`PL1`) | A8 (`PK0`) | Y-min (`PJ1`) | 50:1 Planetary | **444.444444** | $+65^\circ \dots +290^\circ$ |
| **J4** | Forearm | D26 (`PA4`) | D28 (`PA6`) | D24 (`PA2`) | Y-max (`PJ0`) | 805:180 Pulley | **39.753086** | $-240^\circ \dots +180^\circ$ |
| **J5** | Wrist Pitch | D36 (`PC1`) | D34 (`PC3`) | D30 (`PC7`) | Z-min (`PD3`) | 480:109 Pulley | **39.143731** | $-135^\circ \dots +150^\circ$ |
| **J6** | Wrist Roll | D40 (`PG1`) | D41 (`PG0`) | A9 (`PK1`) | Z-max (`PD2`) | 1:1 Direct | **8.888889** | $-180^\circ \dots +360^\circ$ |
| **Grip** | RC Servo | D11 (`PB5`) | — | — | — | Timer 3 PWM | 500–2500 µs | 0–180° / 62.5–0 mm |

---

## 4. Persistent Homing Offset Trims

Physical limit switches have mechanical click hysteresis and mounting variations. Instead of altering firmware constants and reflashing:
1. Trims are stored on the Pi in [`daemon/offsets.json`](file:///c:/Users/mseij/Desktop/PAROLC_Firmware/daemon/offsets.json):
   ```json
   {
     "homing_trims_deg": {
       "j1": 0.0,
       "j2": 0.0,
       "j3": 0.0,
       "j4": 0.0,
       "j5": 0.0,
       "j6": 0.0
     }
   }
   ```
2. The daemon applies trims dynamically to telemetry and kinematics:
   $$\theta_j = \left(\frac{\text{steps}_j}{\text{STEPS\_PER\_DEG}_j}\right) + \text{trim}_j$$
3. Operators can adjust trims directly from the **Homing Calibration Sub-Panel** on the touchscreen kiosk using `+0.1°`, `-0.1°`, `+1.0°`, `-1.0°` quick-step buttons and press **Save Trims**.

---

## 5. Directory Structure & File Map

```
PAROLC_Firmware/
├── Deploy-To-Pi.bat                       # 1-click Windows deployment script
├── gemini-code-starting prompt.md         # Master specification & engineering prompt
├── current printer.cfg                    # Baseline reference configuration
├── daemon/
│   ├── robot_engine.py                    # Master trajectory, kinematics & WebSocket daemon
│   ├── robot_daemon.py                    # Backward compatibility alias to robot_engine.py
│   ├── kinematics.py                      # DH Forward & LM Inverse Kinematics solver
│   ├── trajectory_planner.py              # 100 Hz trapezoidal motion generator
│   ├── mcu_sync.py                        # Linear clock drift estimator & time syncer
│   ├── offsets.json                       # Persistent homing offset trims
│   ├── parol6-daemon.service              # Systemd unit for trajectory daemon
│   ├── parol6.service                     # Alternate service alias
│   └── requirements.txt                   # Python dependencies (websockets, pyserial, numpy)
├── firmware/
│   ├── parol6_core/                       # Active AVR core sketch
│   │   ├── parol6_core.ino                # Serial packet dispatch & setup
│   │   ├── ramps_pins.h                   # Direct AVR register pin definitions
│   │   ├── step_engine.h / .cpp           # Timer 1 virtualized 32-bit event scheduler
│   │   └── homing.h / .cpp                # Soft-start homing state machine
│   ├── parol6_mcu/                        # Synchronized firmware copy
│   └── build/                             # Compiled .hex and .elf binaries
├── ui/
│   └── touchscreen.html                   # High-contrast Chromium touchscreen kiosk UI
├── deploy/
│   ├── deploy_robot.ps1                   # Full PowerShell compilation & deploy pipeline
│   ├── deploy_robot.sh                    # Linux automated installation script
│   ├── remote_install.sh                  # Pi target service & dependency provisioner
│   ├── launch-kiosk.sh                    # X11 kiosk display launcher script
│   └── parol6-kiosk.service               # Systemd unit for kiosk display
├── docs/
│   ├── API_SPEC.md                        # Complete serial & WebSocket protocol specification
│   └── WINDOWS_CONTROL_SUITE_MANUAL.md    # Windows C# WPF real-time joint streaming manual
└── urdf_model/                            # PAROL6 geometric model & joint specs
```

---

## 6. Deployment & Flashing Instructions

### One-Click Deployment from Windows
From the root of `PAROLC_Firmware`:
```cmd
Deploy-To-Pi.bat
```
*Or directly via PowerShell:*
```powershell
powershell -ExecutionPolicy Bypass -File deploy\deploy_robot.ps1 -TargetHost "192.168.0.159" -TargetUser "mark"
```
This automated script:
1. Compiles `firmware/parol6_core/` using `arduino-cli` for `arduino:avr:mega`.
2. Syncs daemon scripts, UI, and compiled binaries to `/home/mark/parol6/` on the Pi via SCP.
3. Stops the active daemon service to release the serial port `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`.
4. Flashes the ATmega2560 via `avrdude` at 115,200 baud (`-c wiring -p m2560`).
5. Restarts `parol6-daemon.service` and `parol6-kiosk.service`.
6. Executes automated health check queries to verify WebSocket and HTTP endpoints.

### Remote Diagnostics on the Raspberry Pi
Connect via SSH:
```bash
ssh mark@192.168.0.159
```
Check daemon status:
```bash
systemctl status parol6-daemon
journalctl -u parol6-daemon -f
```
Check touchscreen kiosk status:
```bash
systemctl status parol6-kiosk
```

---

## 7. Connecting from External Applications

### WebSocket Endpoint:
`ws://<pi_ip>:8765` (e.g. `ws://192.168.0.159:8765`)

### Touchscreen UI in Browser:
`http://<pi_ip>:8000` (e.g. `http://192.168.0.159:8000`)

### Quick Python Test:
```python
import asyncio, websockets, json

async def main():
    async with websockets.connect("ws://192.168.0.159:8765") as ws:
        # Query persistent trims
        await ws.send(json.dumps({"cmd": "GET_TRIMS"}))
        print("Trims:", await ws.recv())

asyncio.run(main())
```

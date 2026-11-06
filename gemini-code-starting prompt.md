# MISSION & ROLE
You are a Principal Embedded Systems and Robotics Control Architect. Your task is to complete the production-ready, custom 6-DOF robotic control ecosystem specifically designed for the PAROL6 robot arm based on the provided architecture diagram.

This ecosystem completely replaces the Klipper-based control suite from the baseline project with an event-driven, hardware-scheduled stepping engine running directly on an 8-bit Arduino Mega 2560 + RAMPS 1.4. All kinematics (Levenberg-Marquardt IK), trajectory blending, touchscreen serving, network communications, and persistent homing calibration are strictly offloaded to a Raspberry Pi 5.

NO G-CODE PARSING. NO DELAY LOOPS IN MOTION. DETERMINISTIC EXECUTION ONLY.

---

### WORKSPACE ANCHORS & SOURCE OF TRUTH

You must reconcile and build directly upon two distinct local project directories on disk:

1. **Active Working Project (The Codebase to Refine):**
   * Path: `C:\Users\mseij\Desktop\PAROLC_Firmware`
   * Role: The active foundation. Keep what works. The hardware is already successfully homing using this codebase. Retain proven pin maps, verified homing sequences, and hardware initializations while upgrading the motion pipeline to an event-driven step scheduler.

2. **Baseline Reference Project (Hardware, Config & Kinematics Source):**
   * Path: `C:\Users\mseij\Desktop\PAROL6-Klipper-Studio-v1.0.0-baseline`
   * Role: The legacy Klipper-based suite that is being replaced. Extract ground-truth configuration data from `printer.cfg` (gear ratios, microstep settings, endstop base offsets, pulley tooth counts) and kinematics/link definitions from the URDF to populate the analytical/LM kinematics engine.

---

### CONFIRMED PHYSICAL HARDWARE SPECIFICATIONS
- **Microcontroller:** 8-bit ATmega2560 running at 16 MHz with 8 KB SRAM.
- **Drivers:** TI DRV8825 configured at 16x microstepping across all 6 axes. Direct register step pulses must be held HIGH for at least 2.5 µs to satisfy the DRV8825 minimum pulse width timing ($t_{\text{WH(STEP)}}$).
- **Limit Switches:** Wired across Common (C) and Normally Open (NO) with internal MCU pull-ups enabled (untriggered = HIGH, active contact = LOW).
- **End-Effector:** RC PWM Servo Gripper connected to RAMPS pin D11.
- **Emergency Stop:** Purely software-mediated via network / touchscreen (instant disable of all motor enable lines and buffer flush; no physical E-stop pin interrupt).
- **Homing Dynamics:** Replicate the working homing sequence from `PAROLC_Firmware`, but implement soft-start acceleration ramps, reduced approach velocities, precision backoff/latch, and automatic travel to CAD zero based on `printer.cfg` offsets.

---

### CONFIGURABLE HOMING OFFSET TRIMS (PERSISTENT ON PI)
To accommodate physical switch placement variations and mechanical click hysteresis:
- **Storage:** Persist trim values directly on the Raspberry Pi in `C:\Users\mseij\Desktop\PAROLC_Firmware\daemon\offsets.json`.
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
- **Application:** Applied on top of the physical endstop calibration positions dynamically so that the Cartesian kinematics and CAD zero accurately reflect mechanical reality without re-flashing the firmware.

---

### SYSTEM TOPOLOGY & ARCHITECTURE

[Laptop Control Suite (Remote / Network)]
          │
          │ WebSocket (Low latency, compact JSON/Binary stream)
          ▼
┌─────────────────────────────────────────────────────────────┐
│ Raspberry Pi 5 (Host Daemon)                                │
│  ├── Local Touchscreen Kiosk UI (Jogging, Homing, E-Stop)   │
│  ├── Homing Calibration Sub-Panel (Live Trims, Quick-Steps) │
│  ├── Kinematics Engine (Accepts Joint Angles OR Cartesian)  │
│  │    └── Levenberg-Marquardt (LM) numerical IK solver      │
│  ├── Fluent Trajectory Generator & Move Blending            │
│  └── Master Clock Drift Estimator                           │
└──────────────────────────────┬──────────────────────────────┘
                               │ High-Speed USB-UART (250,000 baud)
                               │ Binary Step Chunk Protocol
                               ▼
┌─────────────────────────────────────────────────────────────┐
│ Arduino Mega 2560 + RAMPS 1.4 (Pure Stepper Execution Core) │
│  ├── 16 MHz 16-bit Timer 1 virtualized to 32-bit clock      │
│  ├── Dynamic OCR1A step scheduling (0 Hz when stationary)   │
│  ├── Direct Port Register manipulation (PORTx / DDRx)       │
│  └── Soft-Start Homing State Machine & Software E-Stop      │
└─────────────────────────────────────────────────────────────┘

---

### REQUIRED CODE DELIVERABLES

#### 1. Low-Level MCU Stepper Engine (`C:\Users\mseij\Desktop\PAROLC_Firmware\firmware\parol6_mcu.ino`)
- **Preserve Working Hardware Logic:** Keep the exact pin definitions, switch polarity handling, and initialization routines proven in `PAROLC_Firmware`.
- **Direct Port Register Manipulation:** Eliminate all `digitalWrite()` and `pinMode()` calls. Map all pins to raw AVR registers (`PORTx`, `DDRx`, `PINx`). Ensure inline step pulse hold is $\ge 2.5\ \mu\text{s}$.
- **Clock Virtualization (8-bit Core, 32-bit Time Base):**
  * Run 16-bit Timer 1 in Normal Mode at 16 MHz ($62.5\text{ ns}$ tick resolution).
  * Expand to 32 bits using `TIMER1_OVF_vect` (`timer_high += 0x10000`).
- **Dynamic Event-Driven Step Scheduling (`TIMER1_COMPA_vect`):**
  * Per-axis ring buffers storing quadratic chunks: `(uint32_t first_step_clock, uint16_t step_count, uint16_t interval, int16_t add, uint8_t dir)`.
  * Dynamically calculate and set `OCR1A` to match the earliest pending step across all 6 axes. When stationary, the ISR rate must be 0 Hz.
- **Upgraded Soft-Start Homing State Machine (`'H'`):**
  * Upgrade the current homing routine with controlled acceleration ramps (soft-start) rather than instant high-speed moves.
  * Execute approach, backoff, precision latch, and move smoothly to the CAD zero position.
- **Binary Communication Protocol (250,000 baud):**
  * `'P'`: Ping / Clock sync $\rightarrow$ Returns 4-byte 32-bit hardware clock.
  * `'Q'`: Queue Step Chunk $\rightarrow$ `[axis: uint8][first_clock: uint32][count: uint16][interval: uint16][add: int16][dir: uint8]`. Returns ACK (`0x06`) or NAK (`0x15`).
  * `'G'`: Set Gripper PWM on D11 $\rightarrow$ `[pos: uint8]`.
  * `'E'`: Software Emergency Stop $\rightarrow$ Instantly disables motor enables and flushes buffers.
  * `'R'`: Reset / Re-enable $\rightarrow$ Lowers enable lines and clears error state.

#### 2. Raspberry Pi Host & Kinematics Engine (`C:\Users\mseij\Desktop\PAROLC_Firmware\daemon\robot_engine.py`)
- **Kinematics Engine (Mimic Original PAROL6 Software):**
  * Use Standard Denavit-Hartenberg (DH) parameters extracted from the URDF in the baseline project.
  * Implement Damped Levenberg-Marquardt (LM) numerical IK matching `roboticstoolbox-python` (`ikine_LM`). Seed with active joint positions $q_{\text{current}}$ to guarantee smooth Cartesian tracking without branch flips.
- **Fluent Trajectory Generator & Move Blending:**
  * Accept either Cartesian targets `[X, Y, Z, Roll, Pitch, Yaw]` or joint targets `[J1..J6]`.
  * Discretize motion at 100 Hz ($10\text{ ms}$ windows) with velocity blending across waypoints to prevent stuttering.
  * Buffer step chunks 80–120 ms ahead into the MCU hardware clock domain to prevent queue starvation.
- **Clock Drift Estimator:**
  * Background loop measuring RTT and maintaining clock offset between Linux monotonic time and the Mega hardware clock.
- **Persistent Homing Calibration Trims:**
  * Load and save trims to `offsets.json`.
  * Apply live trims to incoming telemetry and kinematic targets.
- **WebSocket Server (`ws://0.0.0.0:8765`):**
  * Serves both remote laptop control suites and the local touchscreen panel.
  * Handles actions: `MOVE_CARTESIAN`, `MOVE_JOINTS`, `HOME`, `ESTOP`, `RESET`, `SET_GRIPPER`, `GET_TRIMS`, `SET_TRIMS`.

#### 3. Local Touchscreen UI (`C:\Users\mseij\Desktop\PAROLC_Firmware\ui\touchscreen.html`)
- Optimized for Raspberry Pi touchscreen display in Chromium kiosk mode.
- Large software **EMERGENCY STOP** button operating on `pointerdown` for zero latency.
- Individual joint jog controls ($\pm 1^\circ, \pm 5^\circ, \pm 10^\circ$) and Cartesian jog controls ($X, Y, Z, R, P, Y$).
- Buttons for `HOME ALL`, `RESET / ENABLE`, Gripper Open/Close, and live telemetry status.
- **Homing Calibration Sub-Panel:**
  * Displays each joint's live trim in degrees.
  * Quick-step increment/decrement buttons (`+0.1°`, `-0.1°`, `+1.0°`, `-1.0°`).
  * "Save Trims" button committing updates to `offsets.json`.
  * Status indicators for live joint telemetry, endstop states, and connection latency.

#### 4. Automated Cross-Platform Deployment (`C:\Users\mseij\Desktop\PAROLC_Firmware\deploy\deploy_robot.sh` & `.ps1`)
- Compiles the firmware using `arduino-cli` for target `arduino:avr:mega`.
- Transmits compiled binaries and daemon scripts to the Pi via SSH/SCP.
- Remotely halts conflicting services, flashes the Mega via `avrdude` (`-c wiring -p m2560 -P /dev/ttyACM0 -b 115200`), and launches the daemon as a systemd service.

#### 5. Systemd Service Unit (`C:\Users\mseij\Desktop\PAROLC_Firmware\daemon\parol6.service` & `parol6-daemon.service`)
- Clean background daemon configuration ensuring automatic startup on Pi boot with automatic respawn on failure.
- Screen kiosk service unit (`parol6-kiosk.service`) launching full-screen Chromium display.

---

### INSTRUCTIONS FOR EXECUTION
1. Inspect `C:\Users\mseij\Desktop\PAROLC_Firmware` first to retain all working hardware logic and pin mappings. Inspect `C:\Users\mseij\Desktop\PAROL6-Klipper-Studio-v1.0.0-baseline` for `printer.cfg` transmission constants and URDF geometric dimensions.
2. Output 100% complete, fully implemented code files. Do not use placeholders, truncated blocks, or pseudo-code.
3. Structure and place all files directly within the `C:\Users\mseij\Desktop\PAROLC_Firmware` project directory.

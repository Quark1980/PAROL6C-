# PAROL6C Architecture & API Specification

> **Complete Protocol Reference for the Custom 6-Axis Klipper PAROL6 Ecosystem**  
> Windows .NET 8 3D Control Suite $\longleftrightarrow$ Raspberry Pi 5 Brain Daemon (`robot_engine.py`) $\longleftrightarrow$ 6-Axis Klipper (`cartesian_abc`) $\longleftrightarrow$ Arduino Mega 2560 + RAMPS 1.4

---

## 1. System Topology

```text
┌───────────────────────────────────────────────────────────────────┐
│        Windows 11 Control Suite (.NET 8 WPF + HelixToolkit)       │
│  - 3 Modes: Simulate Only | Send On Release | Live Follow (50 Hz) │
│  - 2 IK Modes: Keep Tool Steady (Locked R0) | Free Wrist (Probe)  │
│  - Analytical Spherical Wrist + DLS IK & 3.0° Delta Safety Gate   │
└───────────────┬───────────────────────────────────┬───────────────┘
                │ TCP WebSocket (:8765 JSON)        │ Raw UDP (:8766 Binary)
                ▼                                   ▼
┌───────────────────────────────────────────────────────────────────┐
│            Raspberry Pi 5 Host Daemon (robot_engine.py)           │
│  - Hybrid Dual-Transport Server (WS :8765, UDP :8766, HTTP :8000) │
│  - 64-Slot Jitter Buffer (100 Hz Extraction) & Synced 6D 1-Euro   │
│  - Parol6Kinematics + validate_and_lock_branch() Safety Gate      │
│  - Stationary-Interlocked RC Servo Gripper Driver                 │
│  - Non-Blocking PTY Klipper Bridge (/tmp/printer)                 │
└─────────────────────────────────┬─────────────────────────────────┘
                                  │ Coordinated G1 X Y Z A B C F
                                  ▼
┌───────────────────────────────────────────────────────────────────┐
│     6-Axis Klipper Core (cartesian_abc.py + Mega 2560 / RAMPS)    │
│  - USB-UART @ 250,000 Baud Microsecond Stepper Scheduling         │
│  - 6x DRV8825 Drivers (J1..J6) + 6x Pullup Endstops               │
│  - Hardware PWM Servo [servo gripper] on RAMPS D11 (PB5)          │
└───────────────────────────────────────────────────────────────────┘
```

---

## 2. Hardware Pinout & Joint Limits (`printer.cfg` & `kinematics.py`)

CAD Home Pose: `CAD_HOME_DEG = [0.0, -90.0, 180.0, 0.0, 0.0, 180.0]`

| Joint | Klipper Axis | Step Pin | Dir Pin | Enable Pin | Endstop Pin | `rotation_distance` | Cable Protection Limits |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **J1** | `stepper_x` | `PF0` | `!PF1` | `!PD7` | `^PE5` | `36.0` ($10:1$) | $[-170.0^\circ, +170.0^\circ]$ |
| **J2** | `stepper_y` | `PF6` | `!PF7` | `!PF2` | `^PE4` | `7.2` ($50:1$) | $[-150.0^\circ, +50.0^\circ]$ |
| **J3** | `stepper_z` | `PL3` | `PL1` | `!PK0` | `^PJ1` | `7.2` ($50:1$) | $[+65.0^\circ, +290.0^\circ]$ |
| **J4** | `stepper_a` | `PA4` | `PA6` | `!PA2` | `^PJ0` | `80.49689` ($805:180$) | $[-160.0^\circ, +160.0^\circ]$ |
| **J5** | `stepper_b` | `PC1` | `PC3` | `!PC7` | `^PD3` | `81.75` ($480:109$) | $[-110.0^\circ, +110.0^\circ]$ |
| **J6** | `stepper_c` | `PG1` | `PG0` | `!PK1` | `^PD2` | `360.0` ($1:1$) | $[0.0^\circ, +360.0^\circ]$ |
| **Grip** | `[servo gripper]` | `PB5` (D11) | — | — | — | `0.0005`–`0.0025` s | $0^\circ\text{--}180^\circ$ ($62.5\text{--}0\text{ mm}$) |

---

## 3. WebSocket JSON RPC Commands (Port 8765)

### 3.1 System & Homing Commands
- **`HOME`**: `{ "cmd": "HOME", "mask": 63 }` — Executes Klipper `HOME_ALL` macro and sets kinematic zero to `CAD_HOME_DEG + trims`.
- **`HOME_JOINT`**: `{ "cmd": "HOME_JOINT", "joint": 1 }` — Homes a single axis (`1`–`6`).
- **`SET_CAD_HOME`**: `{ "cmd": "SET_CAD_HOME" }` — Forces current position to calibrated CAD Home.
- **`ESTOP`**: `{ "cmd": "ESTOP" }` — Immediately disables motors (`M84`), flushes motion queues, and sets state to `ESTOP`.
- **`RESET`**: `{ "cmd": "RESET" }` — Clears `ESTOP`, issues `FIRMWARE_RESTART` if needed, and re-enables steppers (`WAKE_ARM`).
- **`CLEAR_LOGS`**: `{ "cmd": "CLEAR_LOGS" }` — Clears the rolling `comm_logs` ring buffer.

### 3.2 Motion & Gripper Commands
- **`MOVE_JOINTS`**: `{ "cmd": "MOVE_JOINTS", "joints": [0, -90, 180, 0, 0, 180], "speed": 0.5 }`
- **`MOVE_CARTESIAN`**: `{ "cmd": "MOVE_CARTESIAN", "pose": [0.0, 250.0, 200.0, -180.0, 0.0, -90.0], "speed": 0.5 }` — Sub-steps in $\le 1.5\text{ mm}$ linear increments with strict $R_0$ orientation lock and `validate_and_lock_branch()` protection.
- **`SERVO_JOINTS`**: `{ "cmd": "SERVO_JOINTS", "joints": [0, -90, 180, 0, 0, 180], "time": 0.02 }`
- **`SET_GRIPPER`**: `{ "cmd": "SET_GRIPPER", "angle": 90 }` (or `"mm": 31.2`) — Dispatches `SET_SERVO SERVO=gripper ANGLE=<deg>` with stationary interlock and 25 Hz rate limiting.
- **`GET_TRIMS` / `SET_TRIMS`**: Reads or saves per-joint homing calibration trims in `daemon/offsets.json`.

---

## 4. Raw UDP Binary Streaming Protocol (Port 8766)

Fixed 39-byte packed little-endian binary frame (`<BBHQ6fBBB`):

| Byte Offset | Field | Type | Description |
|:---:|:---|:---:|:---|
| `0` | `Header` | `uint8` | `0xAA` |
| `1` | `Opcode` | `uint8` | `0x01` = `SERVO_JOINTS`, `0x02` = `SERVO_CARTESIAN`, `0x03` = `TELEMETRY`, `0x05` = `HEARTBEAT` |
| `2..3` | `Sequence` | `uint16` | Monotonic packet sequence number |
| `4..11` | `Timestamp_ns` | `uint64` | Nanosecond monotonic timestamp |
| `12..35` | `Payload` | `6x float32` | 6 joint angles (deg) or 6 Cartesian coordinates (mm/deg) |
| `36` | `Gripper` | `uint8` | Servo angle (`0`–`180` deg) |
| `37` | `Flags` | `uint8` | State bitmask (`0x01`=ESTOP, `0x02`=HOMING, `0x04`=MOVING, `0x08`=HOMED, `0x10`=STREAMING) |
| `38` | `CRC8` | `uint8` | CRC-8-CCITT checksum (`poly=0x07`) over bytes `0..37` |

# PAROL6 Communication Protocol & Real-Time Motion Specification

---

## 1. Executive Architecture Summary

The PAROL6 6-DOF robotic control ecosystem implements an ultra-low latency, event-driven motion pipeline designed to eliminate trajectory jitter, prevent communication buffer overflows, and enforce strict fail-safe motor shutdown. 

Motion is bifurcated across two deterministic physical boundaries:
1. **Network Transport (Windows Control Suite $\longleftrightarrow$ Raspberry Pi 5 Daemon)**:
   - Transport: Full-duplex WebSocket over Ethernet / 5 GHz Wi-Fi.
   - Real-Time Protocol: Fixed-width **31-byte packed binary ArrayBuffers** operating at 50–100 Hz.
   - Non-Real-Time Protocol: Structured JSON reserved exclusively for one-time initialization, persistent trim queries (`GET_TRIMS`), calibration adjustments, and telemetry metadata.
2. **Serial Transport (Raspberry Pi 5 Daemon $\longleftrightarrow$ ATmega2560 + RAMPS 1.4)**:
   - Transport: Direct USB-UART at **250,000 baud** (8N1).
   - Real-Time Protocol: Fixed-width **11-byte packed binary step-chunk frames** (`CompactStepChunk`) paced to prevent AVR hardware UART FIFO overruns.
   - Non-Real-Time Protocol: Strict newline-terminated ASCII frames for homing sequences, manual single-stepping, and hardware diagnostics.

```
┌────────────────────────────────────────────────────────┐
│         Windows Laptop Test Suite (WPF / .NET 8)       │
│  - Interactive 3D URDF Viewport & TCP Transform Gizmo  │
│  - 50 Hz Packed Binary Streaming Loop (Opcode 0x01)    │
│  - 15 Hz Continuous Binary Heartbeat Loop (Opcode 0x05)│
└───────────────────────────┬────────────────────────────┘
                            │ WebSocket (31-byte Binary ArrayBuffers @ 50-100 Hz)
                            │ [0xAA][Opcode][Seq:u16][Payload:6x f32][Grip][Flags][CRC8]
                            ▼
┌────────────────────────────────────────────────────────┐
│             Raspberry Pi 5 Host Daemon                 │
│  - Real-Time Trajectory Planner (100 Hz Interpolation) │
│  - Levenberg-Marquardt & Damped Least-Squares IK       │
│  - Dynamic Per-Joint Speed & Acceleration Clamping     │
│  - Microsecond Master Clock Estimator & Linear Filter  │
│  - Tier 1 Watchdog: 150 ms Inactivity Controlled Hold  │
│  - Paced UART Chunk Streamer (<= 2 chunks / 5 ms tick) │
└───────────────────────────┬────────────────────────────┘
                            │ USB-UART @ 250,000 Baud (11-byte Frames)
                            │ [0x55][axis_dir:u8][clock:u32][count:u16][interval:u16][add:i8]
                            ▼
┌────────────────────────────────────────────────────────┐
│      ATmega2560 + RAMPS 1.4 Hardware Step Core         │
│  - Virtualized 32-bit Master Clock (16 MHz Timer 1)    │
│  - Dynamic Compare Match OCR1A Hardware Interrupts     │
│  - Direct Register Step Generation (PORTx / DDRx)      │
│  - 2.5 µs Deterministic Pulse Width for TI DRV8825     │
│  - 6x Ring Buffers (64 chunks capacity per joint)      │
│  - Tier 2 Watchdog: 250 ms Inactivity Coil Shutdown    │
│  - Instant Homing Position Invalidation on E-Stop       │
└────────────────────────────────────────────────────────┘
```

---

## 2. WebSocket Binary Protocol (Windows Suite $\longleftrightarrow$ Host Daemon)

### 2.1 Frame Layout (31 Bytes Fixed Width)

All real-time motion and state messages sent across WebSocket utilize a fixed 31-byte binary packet structure with little-endian numerical encoding:

| Byte Offset | Field Name | Data Type | Description |
| :---: | :---: | :---: | :--- |
| `0` | **Header** | `uint8` | Constant synchronization byte: `0xAA` (170 decimal) |
| `1` | **Opcode** | `uint8` | Command / Telemetry operation identifier (see table below) |
| `2..3` | **Sequence** | `uint16_le` | Incremental sequence number (`0` to `65535` with rollover) |
| `4..27` | **Payload** | `6x float32_le` | 24 bytes: 6 single-precision IEEE 754 floats |
| `28` | **Gripper** | `uint8` | Commanded or actual gripper position (`0` to `180` degrees) |
| `29` | **Flags** | `uint8` | Bitfield representing hardware and motion states |
| `30` | **CRC8** | `uint8` | XOR checksum across bytes `0` to `29` |

### 2.2 Opcode Definitions

| Opcode | Identifier | Direction | Payload Structure |
| :---: | :--- | :---: | :--- |
| `0x01` | `SERVO_JOINTS` | Client $\to$ Host | Commanded target joint angles `[J1, J2, J3, J4, J5, J6]` in **degrees** |
| `0x02` | `SERVO_CARTESIAN`| Client $\to$ Host | Commanded Cartesian tool pose `[X, Y, Z, Rx, Ry, Rz]` (`mm` and `deg`) |
| `0x03` | `TELEMETRY` | Host $\to$ Client | Physical live joint positions `[J1, J2, J3, J4, J5, J6]` in **degrees** |
| `0x05` | `HEARTBEAT` | Bidirectional | Bytes 4..11: Client send timestamp (`double`); Bytes 12..27: Zero padding |

### 2.3 Status Flags Bitmask (Byte 29)

| Bit | Name | Active High (1) | Active Low (0) |
| :---: | :--- | :--- | :--- |
| `0` | `HOMED` | All 6 axes homed & calibrated | Unhomed (motion strictly blocked) |
| `1` | `ESTOP` | Emergency Stop engaged / Coils off | Normal operation |
| `2` | `STREAMING_ACTIVE` | Real-time continuous streaming ON | Staged discrete move mode |
| `3` | `TRAJECTORY_BLOCKED`| IK singular / limit exceeded | Path clear |
| `4..7`| *Reserved* | Reserved for future expansion | Always `0` |

### 2.4 Integrity & Checksum Verification

The CRC8 field (Byte 30) validates transmission integrity against network bit corruption:
$$\text{CRC8} = \bigoplus_{i=0}^{29} \text{Byte}[i]$$
If the calculated checksum does not match Byte 30, the host daemon immediately discards the frame and increments the dropped packet diagnostic counter.

---

## 3. Microcontroller Serial Protocol (USB-UART @ 250,000 Baud)

### 3.1 Compact Step Chunk Frame (`0x55`, 11 Bytes Fixed Width)

To maintain high step frequency without saturating the ATmega2560 UART FIFO, step trajectories are discretized by the host into quadratic acceleration step chunks using the 11-byte `CompactStepChunk` format:

```
 0        1        2..5         6..7        8..9        10
+--------+--------+------------+-----------+-----------+--------+
|  0x55  |axis_dir|start_clock |step_count | interval  |add_scld|
| (uint8)| (uint8)|(uint32_le) |(uint16_le)|(uint16_le)| (int8) |
+--------+--------+------------+-----------+-----------+--------+
```

| Byte Offset | Field | Type | Description |
| :---: | :--- | :---: | :--- |
| `0` | **Header** | `uint8` | Constant sync byte `0x55` (ASCII `'U'`) |
| `1` | **axis_dir** | `uint8` | Bit-packed: `(dir << 7) \| (axis & 0x07)`.<br>Bits 0..2: Joint index (0 = J1, ..., 5 = J6).<br>Bit 7: Direction (1 = Positive, 0 = Negative). |
| `2..5` | **start_clock** | `uint32_le` | Absolute 16 MHz master clock tick when the first step fires |
| `6..7` | **step_count** | `uint16_le` | Total number of step pulses in this quadratic chunk |
| `8..9` | **interval** | `uint16_le` | Clock ticks between initial step pulses ($t_{\text{step}} = \text{interval} \times 62.5\text{ ns}$) |
| `10` | **add_scaled** | `int8` | Scaled acceleration delta per step ($\Delta \text{interval} = \text{add\_scaled} \times 4$ ticks) |

#### Transmission Performance Guarantee
At 250,000 baud (1 start bit, 8 data bits, 1 stop bit = 10 bits per byte):
- Byte transfer time: $10 / 250,000 = 40\ \mu\text{s}$ per byte.
- Total chunk transfer time: $11 \times 40\ \mu\text{s} = 440\ \mu\text{s}$ ($0.44\text{ ms}$).
This completes well under the mandatory 0.5 ms budget per active joint, enabling 6-axis coordinated streaming with negligible bus utilization.

### 3.2 Hardware UART Buffer Flow Control & Pacing

The ATmega2560 hardware UART receive buffer (`SERIAL_RX_BUFFER_SIZE`) is strictly **64 bytes**. 

> [!CAUTION]
> **UART FIFO Overflow Vulnerability**:
> Sending an unthrottled burst of trajectory chunks (e.g., 72 chunks = 792 bytes) instantly overflows the 64-byte FIFO, resulting in dropped bytes. A dropped byte causes frame desynchronization, shifting payload bytes into the opcode position. If a random payload byte matches an ASCII control command (`'D'`, `'H'`, `'I'`), it triggers false direction inversion or uncommanded wrist homing, freezing the MCU.

#### Mandatory Host Pacing Rules:
1. **Burst Throttling**: The trajectory streamer in `daemon/trajectory_planner.py` restricts serial dispatches to **a maximum of 2 chunks (22 bytes) per 5 ms tick cycle**.
2. **Buffer Headroom**: 22 bytes consumes only 34% of the 64-byte buffer, leaving 42 bytes free for incoming ping requests and emergency stop tokens.
3. **Strict Trailing Newline Enforcment**: Every single-byte ASCII maintenance command (`'H'`, `'I'`, `'D'`, `'M'`, `'G'`) requires a trailing newline (`\n` or `\r\n`). If an isolated byte arrives without a trailing newline, it is discarded, completely immunizing the MCU from false ASCII triggers caused by corrupted binary data.

---

## 4. Dual-Tier Watchdog & Fail-Safe Architecture

The ecosystem implements two independent, hierarchically separated watchdogs to handle network disconnects, host operating system stalls, and communication line breaks:

```
┌────────────────────────────────────────────────────────┐
│                  Windows Control Suite                 │
└───────────────────────────┬────────────────────────────┘
                            │ Continuous Heartbeat (15 Hz)
                            ▼
┌────────────────────────────────────────────────────────┐
│              Tier 1 Watchdog (Host Daemon)             │
│  Timeout: 150 ms                                       │
│  Trigger: Heartbeat loss during active streaming       │
│  Action: Controlled Deceleration to HOLD Stop          │
│  State: Preserves Homing Calibration & Position        │
└───────────────────────────┬────────────────────────────┘
                            │ Continuous Step Chunks / Pings (>= 10 Hz)
                            ▼
┌────────────────────────────────────────────────────────┐
│              Tier 2 Watchdog (ATmega2560 MCU)          │
│  Timeout: 250 ms                                       │
│  Trigger: Serial bus silence from Host Daemon          │
│  Action: Immediate Coil De-energization (Hardware Off) │
│  State: Clears homed_mask = 0 (Position Invalidated)   │
└────────────────────────────────────────────────────────┘
```

### 4.1 Tier 1: Client-to-Host Watchdog (150 ms)
- **Monitoring Location**: Raspberry Pi 5 Host Daemon (`client_watchdog_loop`).
- **Heartbeat Rate**: Windows Suite broadcasts binary `HEARTBEAT` (`0x05`) at **15 Hz** (every 66 ms).
- **Trigger Condition**: Continuous streaming is active (`is_streaming_active == True`) and no packet has arrived for $> 150\text{ ms}$.
- **Fail-Safe Response**:
  1. Daemon halts incoming trajectory generation.
  2. Trajectory planner issues smooth S-curve deceleration to a **controlled HOLD stop** at the current coordinates.
  3. Real-time streaming flag is cleared (`streaming_active = False`).
  4. **Arm coordinates and homed state are retained**, allowing seamless recovery without re-homing.

### 4.2 Tier 2: Host-to-MCU Hardware Watchdog (250 ms)
- **Monitoring Location**: ATmega2560 Firmware (`TIMER1_OVF_vect` master clock loop).
- **Heartbeat Rate**: Host daemon maintains serial activity (chunks or `'P'` ping frames) at $\ge 10\text{ Hz}$.
- **Trigger Condition**: No valid serial byte or chunk received for $> 250\text{ ms}$ ($4,000,000$ clock cycles at 16 MHz).
- **Fail-Safe Response**:
  1. `handle_estop()` executes immediately.
  2. Direct register writes force all motor `ENABLE` pins HIGH (`PORTD7`, `PORTF2`, `PORTK0`, `PORTA2`, `PORTC7`, `PORTK1`), completely de-energizing all 6 stepper motor coils.
  3. Hardware step scheduling timer (`OCR1A`) is cleared.
  4. Ring buffer pointers and step queues are wiped.
  5. **Critical Safety State**: `homed_mask` is reset to `0x00`. Because de-energized coils cause the arm to drop under gravity (especially J2 and J3), position tracking is permanently invalidated. Motion cannot resume until a full multi-stage hardware homing sequence is executed.

---

## 5. Microsecond Master Clock Synchronization

### 5.1 Virtualized 32-bit Clock Architecture
The ATmega2560 16-bit Timer 1 runs in Normal Mode with Prescaler 1 (16 MHz clock, 62.5 ns per tick). The 16-bit Timer 1 overflow interrupt (`TIMER1_OVF_vect`) maintains an upper 16-bit rollover counter:
$$\text{Clock}_{\text{32}} = (\text{timer1\_overflow\_count} \ll 16) \mid \text{TCNT1}$$
This provides a continuous, drift-free 32-bit hardware clock that wraps every $268.435\text{ seconds}$ ($2^{32} \times 62.5\text{ ns}$).

### 5.2 Host Linear Regression Drift Estimator
The Raspberry Pi 5 runs a background 10 Hz clock sync task querying the MCU with `'P'`:
1. Host records local high-resolution monotonic time $t_{\text{send}}$.
2. MCU immediately returns its current 32-bit `master_clock`.
3. Host records arrival time $t_{\text{recv}}$.
4. Round-Trip Time ($RTT$) and one-way network transit latency are computed:
   $$RTT = t_{\text{recv}} - t_{\text{send}}, \quad t_{\text{transit}} = \frac{RTT}{2}$$
5. A recursive least-squares (RLS) linear regression estimator fits host monotonic time to MCU clock ticks:
   $$\text{TargetClock}(t) = \text{Clock}_{\text{base}} + \text{Frequency}_{\text{ratio}} \cdot (t - t_{\text{base}})$$
6. Trajectory chunks are scheduled with an absolute start clock $50\text{ ms}$ to $100\text{ ms}$ into the future, guaranteeing that steps fire with sub-microsecond synchronization and zero phase jitter.

---

## 6. Endstop & Homing Sequence Truth Table

### 6.1 Limit Switch Specifications
- **Wiring**: Switches wired across Common (C) and Normally Open (NO) with internal MCU pull-ups enabled (`PORTx |= PIN_MASK`).
- **Logic State**:
  - Untriggered (Open): Pin reads `HIGH` (`1`).
  - Triggered (Contact): Pin pulled to GND, reads `LOW` (`0`).
- **Configuration Masks**:
  - `endstop_invert_mask = 0x00` (Default active-low).
  - `dir_invert_mask = 0x03` (J1 and J2 hardware direction inverted).

### 6.2 Multi-Stage Parallel Homing Sequence

To eliminate mechanical pinch points and ensure repeatable zero calibration, homing must execute in the exact verified multi-stage sequence:

| Stage | Target Joints | Direction | Fast Feed | Backoff | Slow Feed | Purpose |
| :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **Stage 1** | **J1 + J2 + J3** | - / - / - | 2000 steps/s | 5° | 400 steps/s | Simultaneous parallel homing of primary heavy links. |
| **Stage 2** | **J4** | - | 1500 steps/s | 5° | 300 steps/s | Forearm roll homing after upper arm is positioned. |
| **Stage 3** | **J6 (Pass 1)** | - | 1000 steps/s | 5° | 200 steps/s | Initial wrist roll reference latch. |
| **Stage 4** | **J5** | - | 1200 steps/s | 5° | 250 steps/s | Wrist pitch homing while J6 is clear. |
| **Stage 5** | **J6 (Pass 2)** | - | 800 steps/s | 5° | 150 steps/s | Final precision wrist roll zeroing in true CAD pose. |
| **Stage 6** | **All Joints** | + / - | Trajectory | N/A | Smooth | Travel to trimmed CAD zero coordinates (`offsets.json`). |

---

## 7. Configuration & Persistent Trims (JSON Schema)

Non-real-time configuration is serialized via standard JSON. Persistent trims fine-tune mechanical switch placement and click hysteresis to achieve $\pm 0.05^\circ$ CAD accuracy:

### `daemon/offsets.json` Schema:
```json
{
  "j1": 0.00,
  "j2": -1.25,
  "j3": 0.50,
  "j4": 0.00,
  "j5": 0.85,
  "j6": -0.40
}
```

### Dynamic Speed Limits Payload (`SET_SPEED_LIMITS`):
```json
{
  "cmd": "SET_SPEED_LIMITS",
  "scale": 0.80,
  "limits": {
    "j1": 60.0,
    "j2": 45.0,
    "j3": 50.0,
    "j4": 90.0,
    "j5": 90.0,
    "j6": 120.0
  }
}
```
All velocity and acceleration clamps are dynamically applied by `daemon/trajectory_planner.py` before trajectory chunk discretization, guaranteeing stepper motors never exceed safe torque curves.

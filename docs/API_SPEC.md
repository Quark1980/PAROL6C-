# PAROL6 6-DOF Robotic Control Ecosystem: Architecture & API Specification

---

## 1. Architectural Overview

The PAROL6 custom control ecosystem replaces traditional Klipper motion planning with an event-driven, absolute hardware-clock step execution engine. The computational responsibilities are cleanly bifurcated between the host processor and microcontroller:

```
┌───────────────────────────────────────────────────────────────┐
│                    Raspberry Pi 5 (Host)                      │
│  - 6-DOF URDF Forward & Inverse Kinematics (DLS Solver)      │
│  - Multi-Axis S-Curve / Trapezoidal Trajectory Planner        │
│  - Step Compressor (Continuous -> Quadratic Step Chunks)      │
│  - Hardware Clock Estimator & Microsecond Sync Loop (10 Hz)   │
│  - Async WebSocket Server (Port 8765, 20 Hz Telemetry)        │
└──────────────────────────────┬────────────────────────────────┘
                               │ USB-UART @ 250,000 Baud
                               ▼
┌───────────────────────────────────────────────────────────────┐
│           ATmega2560 + RAMPS 1.4 (Step Execution Engine)      │
│  - Timer 1: 16 MHz Master Clock (Normal Mode, Prescaler 1)    │
│  - Dynamic Compare Match (OCR1A) Matching Earliest Step Event  │
│  - 6 Independent Ring Buffers (Quadratic Acceleration Chunks) │
│  - Cycle-Accurate Direct Register Port Writes (PORTx, DDRx)   │
│  - Timer 3: 50 Hz Hardware PWM RC Servo Driver (D11 & D4)     │
│  - Non-Blocking Hardware Homing Sequencer (Endstop Latches)   │
└───────────────────────────────────────────────────────────────┘
```

---

## 2. Hardware Port Mappings & Actuator Constants

All pin assignments directly mirror the baseline `printer.cfg` configuration for RAMPS 1.4:

| Axis | Function | ATmega2560 Pin | Direct Port Reg | Invert | Gear Ratio | Steps / Degree |
| :--- | :--- | :--- | :--- | :---: | :---: | :---: |
| **J1** | Base Yaw Step | A0 (PF0) | `PORTF0` | No | 10:1 (GT2 160:16) | 88.88888889 |
| | Base Yaw Dir | A1 (PF1) | `PORTF1` | **Yes** | | |
| | Base Yaw Enable | D38 (PD7) | `PORTD7` | Active Low | | |
| | Base Endstop | X-min (PE5) | `PORTE5` (Pullup) | Active Low | | |
| **J2** | Shoulder Step | A6 (PF6) | `PORTF6` | No | 50:1 Planetary | 444.44444444 |
| | Shoulder Dir | A7 (PF7) | `PORTF7` | **Yes** | | |
| | Shoulder Enable | A2 (PF2) | `PORTF2` | Active Low | | |
| | Shoulder Endstop | X-max (PE4) | `PORTE4` (Pullup) | Active Low | | |
| **J3** | Elbow Step | D46 (PL3) | `PORTL3` | No | 50:1 Planetary | 444.44444444 |
| | Elbow Dir | D48 (PL1) | `PORTL1` | No | | |
| | Elbow Enable | A8 (PK0) | `PORTK0` | Active Low | | |
| | Elbow Endstop | Y-min (PJ1) | `PORTJ1` (Pullup) | Active Low | | |
| **J4** | Forearm Step | D26 (PA4) | `PORTA4` | No | 805:180 Pulley | 39.75308642 |
| | Forearm Dir | D28 (PA6) | `PORTA6` | No | | |
| | Forearm Enable | D24 (PA2) | `PORTA2` | Active Low | | |
| | Forearm Endstop | Y-max (PJ0) | `PORTJ0` (Pullup) | Active Low | | |
| **J5** | Wrist Pitch Step| D36 (PC1) | `PORTC1` | No | 480:109 Pulley | 39.14373089 |
| | Wrist Pitch Dir | D34 (PC3) | `PORTC3` | No | | |
| | Wrist Pitch Enable| D30 (PC7) | `PORTC7` | Active Low | | |
| | Wrist Endstop | Z-min (PD3) | `PORTD3` (Pullup) | Active Low | | |
| **J6** | Wrist Roll Step | D40 (PG1) | `PORTG1` | No | 1:1 Direct | 8.88888889 |
| | Wrist Roll Dir | D41 (PG0) | `PORTG0` | No | | |
| | Wrist Roll Enable| A9 (PK1) | `PORTK1` | Active Low | | |
| | Wrist Endstop | Z-max (PD2) | `PORTD2` (Pullup) | Active Low | | |
| **Grip**| RC Servo Primary| D11 (PB5) | `PORTB5` (Timer 3)| 500–2500 µs (0–180° / 62.5–0 mm gap) |
| | RC Servo Mirror | D4 (PG5) | `PORTG5` (Timer 3)| Mirrored on RAMPS D4 servo output |

---

## 3. Microcontroller Serial Protocol (USB-UART @ 250,000 Baud)

### 3.1 Host-to-MCU Frame Formats

#### Ping / Clock Sync (`'P'`)
- **Payload**: `b'P'` (1 byte)
- **Response**: `b'P' <clock:uint32_le> b'\n'` (6 bytes total)
- **Purpose**: Host records pre-send and post-receive monotonic timestamps to calculate RTT and update the linear time synchronization estimator.

#### Fast Step Chunk Packet (`'Q'`) — Recommended Low-Overhead Streaming Protocol
- **Total Length**: 12 bytes binary
- **Layout**:
  ```
  Offset 0:      'Q' (0x51)
  Offset 1:      axis (uint8, 0 to 5)
  Offset 2..5:   first_clock (uint32_le, 16 MHz master clock tick)
  Offset 6..7:   count (uint16_le, number of steps in this chunk)
  Offset 8..9:   interval (uint16_le, clock ticks between initial steps)
  Offset 10..11: add (int16_le, tick acceleration interval delta per step)
  Offset 12:     dir (uint8, 0 or 1)
  ```
- **Responses**:
  - `0x06` (`ASCII ACK`, 1 byte): Chunk accepted into axis ring buffer.
  - `0x15` (`ASCII NAK`, 1 byte): Chunk rejected (buffer full, unhomed arm, or invalid axis).
- **Driver Timing Guarantee**: Direct register step pulse toggles hold HIGH for $\ge 2.5\ \mu\text{s}$ using cycle-accurate inline assembly delay (`__builtin_avr_delay_cycles(42)`) to satisfy the TI DRV8825 minimum pulse width timing ($t_{\text{WH(STEP)}}$).

#### Legacy Step Chunk Packet (`'S'`)
- **Total Length**: 14 bytes binary
- **Layout**:
  ```
  Offset 0:      'S' (0x53)
  Offset 1:      joint (uint8, 0 to 5)
  Offset 2..5:   first_step_clock (uint32_le, 16 MHz master tick)
  Offset 6..7:   step_count (uint16_le, number of steps in chunk)
  Offset 8..9:   interval (uint16_le, ticks between step 0 and 1)
  Offset 10..11: add (int16_le, interval delta per step)
  Offset 12:     dir (uint8, 0 or 1)
  Offset 13:     crc (uint8, XOR sum of bytes 0 through 12)
  ```
- **Responses**:
  - `b'K' <joint:uint8> <free_slots:uint8> b'\n'` (Chunk Accepted)
  - `b'N' <joint:uint8> <error_code:uint8> b'\n'` (Chunk Rejected: 1=Full, 2=CRC, 3=Joint, 4=ESTOP, 5=Arm Not Homed)
- **Mandatory Safety Interlock**: The microcontroller strictly rejects any step chunks with error code `0x05` / `NAK` unless the hardware homing sequence has completed successfully (`homed_mask == 0x3F`).

#### Emergency Stop (`'E'`)
- **Payload**: `b'E'` (1 byte)
- **Response**: `b'E' \x01 b'\n'`
- **Behavior**: Instantly sets all enable pins HIGH (de-energizing coils), flushes all joint ring buffers, aborts homing.

#### Reset / Wake Motors (`'R'`)
- **Payload**: `b'R'` (1 byte)
- **Response**: `b'R' \x01 b'\n'`
- **Behavior**: Drives all enable pins LOW (energizing motor coils) and resets faults.

#### Gripper Command (`'G'`)
- **Payload**: `b'G' <pulse_us:uint16_le> b'\n'` (4 bytes total) OR `b'G' <pos:uint8>` (2 bytes binary: 0=closed, 255=open)
- **Response**: `b'G' \x01 b'\n'`

#### Soft-Start Homing Trigger (`'H'`)
- **Payload**: `b'H' <joint_mask:uint8> b'\n'` (3 bytes total, `0xFF` = all axes)
- **Response**: `b'H' \x01 b'\n'` on start; broadcasts completion when finished.
- **State Machine Architecture**:
  1. **Order of Operation**:
     - **Stage 1 (Parallel Simultaneous)**: Concurrent stepping of `[J1 + J2 + J3]` (`HOMING_STAGE_J123_PARALLEL`). All three joints seek and latch their switches simultaneously, drastically reducing homing duration while eliminating mechanical collision risks.
     - **Stage 2 (Forearm)**: `J4` (`HOMING_STAGE_J4`) moves $+270^\circ$, latches, and retreats to $-260.0^\circ$.
     - **Stage 3 (Wrist Pass 1)**: `J6` (`HOMING_STAGE_J6_PASS1`) moves $+360^\circ$, latches, and retreats to $-4.0^\circ$.
     - **Stage 4 (Wrist Pitch)**: `J5` (`HOMING_STAGE_J5`) pre-moves `J6` by $-10.0^\circ$ (to clear optical flags and cabling), moves `J5` by $-360^\circ$, latches, retreats to $+120.0^\circ$, and post-moves `J6` by $-5.0^\circ$.
     - **Stage 5 (Wrist Pass 2)**: `J6` (`HOMING_STAGE_J6_PASS2`) homes a second time to eliminate the cumulative $-15.0^\circ$ displacement incurred during J5's clearance maneuvers, precision-latching and retreating to $-4.0^\circ$.
     - **Stage 6 (CAD Zero Reference)**: Physical step positions are locked directly to the PAROL6 CAD zero reference ($J_1=0^\circ, J_2=-90^\circ, J_3=180^\circ, J_4=0^\circ, J_5=0^\circ, J_6=180^\circ$) and the safety interlock is unlocked (`homed_mask == 0x3F`).
  2. **Soft-Start Ramp**: Initial acceleration ramp starting at $5.0^\circ/\text{s}$ ramping smoothly to `seek_speed` (prevents mechanical jerk and tooth skipping).
  3. **Switch Release**: On first contact, backs off until limit switch opens (`SUBSTATE_LATCH_RELEASE`).
  4. **Precision Latch**: Re-approaches switch at $3.0^\circ/\text{s}$ with 5-sample digital debounce filter (`SUBSTATE_LATCH_PRECISION`).
  5. **CAD Zero Retreat**: Smoothly accelerates and decelerates to the configured `retreat_pos` based on baseline `printer.cfg` calibration constants.

#### Telemetry Query (`'?'`)
- **Payload**: `b'?'` (1 byte)
- **Response**: `b'!' <flags:uint8> <pos:int32_le * 6> <endstops:uint8> <clock:uint32_le> b'\n'` (32 bytes)

---

## 4. Host WebSocket API (Port 8765)

The Python daemon hosts a WebSocket server on `ws://<pi_ip>:8765`.

### 4.1 Inbound JSON Client Commands

#### 1. Instant Emergency Stop
```json
{"cmd": "ESTOP"}
```

#### 2. Reset & Re-Energize Drivers
```json
{"cmd": "RESET"}
```

#### 3. Home Sequencer
```json
{"cmd": "HOME", "joint": "ALL"}
```
*(Options for `"joint"`: `"ALL"`, `"WRIST"`, `"J1"`, `"J2"`, `"J3"`, `"J4"`, `"J5"`, `"J6"`)*

#### 4. Single-Joint Jog
```json
{
  "cmd": "JOG_JOINT",
  "joint": 1,
  "delta_deg": 5.0,
  "speed": 0.5
}
```

#### 5. Real-Time Joint Stream (`SERVO_JOINTS` / `STREAM_JOINTS`)
Continuous 20 Hz to 50 Hz streaming command from Windows Control Suite or Touchscreen:
```json
{
  "cmd": "SERVO_JOINTS",
  "joints": [10.5, -85.2, 175.0, 5.0, -2.5, 180.0],
  "time": 0.05
}
```
*(Requires `homed == true`. If unhomed, returns `{"type": "ACK", "cmd": "SERVO_JOINTS", "status": "REJECTED"}`)*

#### 6. Dense Multi-Point Trajectory Upload (`STREAM_TRAJECTORY`)
```json
{
  "cmd": "STREAM_TRAJECTORY",
  "points": [
    { "joints": [0.0, -90.0, 180.0, 0.0, 0.0, 180.0], "time": 0.02 },
    { "joints": [0.5, -89.5, 180.2, 0.1, 0.0, 180.0], "time": 0.02 }
  ]
}
```

#### 7. Multi-Joint Synchronized Move
```json
{
  "cmd": "MOVE_JOINTS",
  "joints": [0.0, -90.0, 180.0, 0.0, 0.0, 180.0],
  "speed": 0.6
}
```

#### 6. Cartesian Jog
```json
{
  "cmd": "JOG_CARTESIAN",
  "axis": "Z",
  "delta": 10.0,
  "speed": 0.4
}
```
*(Supported axes: `"X"`, `"Y"`, `"Z"`, `"RX"`, `"RY"`, `"RZ"`)*

#### 7. Cartesian Move (IK)
```json
{
  "cmd": "MOVE_CARTESIAN",
  "pose": [250.0, 0.0, 150.0, 0.0, 90.0, 0.0],
  "speed": 0.4
}
```

#### 8. Gripper Actuation
```json
{"cmd": "SET_GRIPPER", "mm": 35.0}
```
*(Or by angle: `{"cmd": "SET_GRIPPER", "angle": 90.0}`)*

#### 9. Preset Pose
```json
{"cmd": "PRESET", "name": "CAD_HOME", "speed": 0.5}
```
*(Presets: `"CAD_HOME"`, `"PARK"`, `"CANDLE"`, `"STANDBY"`)*

#### 10. Query Calibration Trims (`GET_TRIMS`)
```json
{"cmd": "GET_TRIMS"}
```
- **Daemon Response**:
  ```json
  {
    "type": "TRIMS",
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

#### 11. Update Persistent Calibration Trims (`SET_TRIMS`)
To accommodate physical switch placement variations and mechanical click hysteresis without re-flashing AVR firmware:
```json
{
  "cmd": "SET_TRIMS",
  "trims": {
    "j1": 0.5,
    "j2": -0.2
  },
  "save": true
}
```
- **Parameters**:
  - `trims` *(object, required)*: Key-value map of joint names (`"j1"` .. `"j6"`) to trim offsets in degrees.
  - `save` *(boolean, optional, default: `true`)*: If `true`, writes immediately to persistent storage (`/home/mark/parol6/daemon/offsets.json` on the Raspberry Pi).
- **Daemon Response**:
  ```json
  {
    "type": "ACK",
    "cmd": "SET_TRIMS",
    "status": "OK",
    "homing_trims_deg": {
      "j1": 0.5,
      "j2": -0.2,
      "j3": 0.0,
      "j4": 0.0,
      "j5": 0.0,
      "j6": 0.0
    }
  }
  ```
- **Kinematic Trim Formulation**:
  The calibrated joint angle transmitted to clients and used in kinematics is computed directly as:
  $$\theta_{\text{calibrated}} = \left(\frac{\text{raw\_mcu\_steps}}{\text{STEPS\_PER\_DEG}}\right) + \text{trim}_{\text{deg}}$$
  Relative joint deltas $\Delta \theta$ during trajectory generation and streaming are invariant to the trim offset, ensuring that trajectory execution remains smooth while CAD zero aligns precisely with the physical robot arm.

#### 12. Reset Hardware Invert Masks (`RESET_INVERT_MASKS`)
Restores motor direction and limit switch polarity masks to the ground-truth baseline (`current printer.cfg`):
```json
{"cmd": "RESET_INVERT_MASKS"}
```
- **Daemon Action**:
  - Sets `dir_invert_mask` to `0x03` (inverting J1 and J2 only).
  - Sets `endstop_invert_mask` to `0x00` (normal active-high pulled-up switches).
  - Immediately transmits `'D'` and `'I'` configuration frames to the ATmega2560 via USB-UART.
- **Daemon Response**:
  ```json
  {
    "type": "ACK",
    "cmd": "RESET_INVERT_MASKS",
    "status": "OK",
    "dir_invert_mask": 3,
    "endstop_invert_mask": 0
  }
  ```

#### 13. Dynamic Polarity Configuration (`SET_DIR_INVERT` / `SET_ENDSTOP_INVERT`)
Allows dynamic reconfiguration of direction or endstop masks from diagnostic dashboards:
```json
{"cmd": "SET_DIR_INVERT", "mask": 3}
{"cmd": "SET_ENDSTOP_INVERT", "mask": 0}
```

#### 14. Query & Configure Joint Speed Limits (`GET_SPEED_LIMITS` / `SET_SPEED_LIMITS`)
Enforces independent maximum velocity clamping across all 6 axes to prevent stepper motor stall under high-rate streaming or aggressive operator jogging:
```json
{"cmd": "GET_SPEED_LIMITS"}
```
- **Daemon Response**:
  ```json
  {
    "type": "SPEED_LIMITS",
    "speed_limits_deg_s": [120.0, 90.0, 90.0, 150.0, 150.0, 180.0]
  }
  ```
- **Set Speed Limits Command**:
  ```json
  {
    "cmd": "SET_SPEED_LIMITS",
    "speed_limits_deg_s": [100.0, 80.0, 80.0, 120.0, 120.0, 150.0]
  }
  ```
- **Daemon ACK**:
  ```json
  {
    "type": "ACK",
    "cmd": "SET_SPEED_LIMITS",
    "status": "OK",
    "speed_limits_deg_s": [100.0, 80.0, 80.0, 120.0, 120.0, 150.0]
  }
  ```

#### 15. Execute Staged Motion (`MOVE_TO_STAGED`)
Dispatches physical motion to the target configuration interactively staged in the Windows Control Suite 3D Viewport (visualized by the translucent cyan ghost twin):
```json
{
  "cmd": "MOVE_TO_STAGED",
  "joints": [15.0, -80.0, 160.0, 10.0, 30.0, 180.0],
  "speed": 0.5
}
```
- **Daemon Response**:
  ```json
  {
    "type": "ACK",
    "cmd": "MOVE_TO_STAGED",
    "status": "OK"
  }
  ```

---

### 4.2 Outbound Broadcast Telemetry (20 Hz)

Broadcasted every 50 ms to all connected clients:

```json
{
  "type": "TELEMETRY",
  "state": "IDLE",
  "joints_deg": [0.0, -90.0, 180.0, 0.0, 0.0, 180.0],
  "pose_mm_deg": [238.5, 0.0, 142.0, 0.0, 90.0, 0.0],
  "gripper_mm": 62.5,
  "homed": true,
  "endstops_mask": 0,
  "endstops": [false, false, false, false, false, false],
  "mcu_clock": 15428930,
  "rtt_ms": 0.42,
  "drift_ppm": 12.4,
  "free_slots": [16, 16, 16, 16, 16, 16],
  "soft_limits": [
    [-175.0, 175.0],
    [-150.0, 50.0],
    [65.0, 290.0],
    [-240.0, 180.0],
    [-135.0, 150.0],
    [-180.0, 360.0]
  ]
}
```

---

## 5. Windows C# / WPF Integration Guide

To control the PAROL6 from a Windows C# .NET 8 / WPF desktop application:

### Complete C# Client Implementation (`Parol6Client.cs`):

```csharp
using System;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Threading;
using System.Threading.Tasks;

public class Parol6Client : IDisposable
{
    private readonly ClientWebSocket _ws = new();
    private readonly CancellationTokenSource _cts = new();

    public event Action<string>? StateChanged;
    public event Action<double[]>? JointPositionsReceived;
    public event Action<double[]>? CartesianPoseReceived;

    public bool IsConnected => _ws.State == WebSocketState.Open;

    public async Task ConnectAsync(string ipAddress, int port = 8765)
    {
        var uri = new Uri($"ws://{ipAddress}:{port}");
        await _ws.ConnectAsync(uri, _cts.Token);
        _ = ReceiveLoopAsync();
    }

    public async Task EmergencyStopAsync()
    {
        await SendJsonAsync(new { cmd = "ESTOP" });
    }

    public async Task ResetAsync()
    {
        await SendJsonAsync(new { cmd = "RESET" });
    }

    public async Task JogJointAsync(int joint, double deltaDeg, double speed = 0.5)
    {
        await SendJsonAsync(new
        {
            cmd = "JOG_JOINT",
            joint = joint,
            delta_deg = deltaDeg,
            speed = speed
        });
    }

    public async Task JogCartesianAsync(string axis, double deltaMm, double speed = 0.5)
    {
        await SendJsonAsync(new
        {
            cmd = "JOG_CARTESIAN",
            axis = axis.ToUpper(),
            delta = deltaMm,
            speed = speed
        });
    }

    public async Task SetGripperAsync(double gapMm)
    {
        await SendJsonAsync(new
        {
            cmd = "SET_GRIPPER",
            mm = gapMm
        });
    }

    private async Task SendJsonAsync(object payload)
    {
        if (_ws.State != WebSocketState.Open) return;
        string json = JsonSerializer.Serialize(payload);
        byte[] bytes = Encoding.UTF8.GetBytes(json);
        await _ws.SendAsync(
            new ArraySegment<byte>(bytes),
            WebSocketMessageType.Text,
            true,
            _cts.Token
        );
    }

    private async Task ReceiveLoopAsync()
    {
        var buffer = new byte[8192];
        while (!_cts.IsCancellationRequested && _ws.State == WebSocketState.Open)
        {
            try
            {
                var result = await _ws.ReceiveAsync(new ArraySegment<byte>(buffer), _cts.Token);
                if (result.MessageType == WebSocketMessageType.Close) break;

                string rawJson = Encoding.UTF8.GetString(buffer, 0, result.Count);
                var doc = JsonNode.Parse(rawJson);
                if (doc?["type"]?.GetValue<string>() == "TELEMETRY")
                {
                    string state = doc["state"]?.GetValue<string>() ?? "UNKNOWN";
                    StateChanged?.Invoke(state);

                    var jNode = doc["joints_deg"]?.AsArray();
                    if (jNode != null && jNode.Count == 6)
                    {
                        double[] joints = new double[6];
                        for (int i = 0; i < 6; i++) joints[i] = (double)jNode[i]!;
                        JointPositionsReceived?.Invoke(joints);
                    }

                    var pNode = doc["pose_mm_deg"]?.AsArray();
                    if (pNode != null && pNode.Count == 6)
                    {
                        double[] pose = new double[6];
                        for (int i = 0; i < 6; i++) pose[i] = (double)pNode[i]!;
                        CartesianPoseReceived?.Invoke(pose);
                    }
                }
            }
            catch (Exception)
            {
                break;
            }
        }
    }

    public void Dispose()
    {
        _cts.Cancel();
        _ws.Dispose();
    }
}
```

---

## 6. Python / Qt Quick-Start Control Script

```python
import asyncio
import json
import websockets

async def test_robot():
    uri = "ws://raspberrypi.local:8765"
    async with websockets.connect(uri) as ws:
        # 1. Wake drivers
        await ws.send(json.dumps({"cmd": "RESET"}))
        print("Drivers awakened.")

        # 2. Jog Joint 1 by +10 degrees
        await ws.send(json.dumps({
            "cmd": "JOG_JOINT",
            "joint": 1,
            "delta_deg": 10.0,
            "speed": 0.4
        }))

        # 3. Read live telemetry response
        async for message in ws:
            telem = json.loads(message)
            if telem.get("type") == "TELEMETRY":
                print(f"State: {telem['state']} | J1: {telem['joints_deg'][0]}° | RTT: {telem['rtt_ms']} ms")
                break

asyncio.run(test_robot())
```

---

## 7. Local Touchscreen Kiosk Interface & Calibration Panel

The Raspberry Pi 5 runs a local Chromium kiosk display interface connected to the physical touchscreen.

### 7.1 Web Application Architecture
- **Host HTTP Server**: Embedded asynchronous HTTP daemon running on `http://0.0.0.0:8000`.
- **Source File**: [`ui/touchscreen.html`](file:///c:/Users/mseij/Desktop/PAROLC_Firmware/ui/touchscreen.html).
- **Zero-Latency Emergency Stop**: Bound directly to the `pointerdown` DOM event (rather than `click`) to ensure 0 ms touch reaction latency. Sends `{"cmd": "ESTOP"}` over WebSocket and triggers MCU coil disable immediately.

### 7.2 Homing Calibration Sub-Panel (`#tab-calibration`)
Designed for precise mechanical alignment without firmware recompilation:
- **Live Calibrated Angle**: Displays live telemetry joint angles with active trims factored in.
- **Current Offset Readout**: Shows active trim in degrees for each joint ($J_1$ through $J_6$).
- **Quick-Step Increment/Decrement Buttons**:
  - `+1.0°` / `-1.0°`: Coarse adjustment.
  - `+0.1°` / `-0.1°`: Fine vernier adjustment.
- **Save Trims**: Sends `SET_TRIMS` with `save: true` to persist updates into [`daemon/offsets.json`](file:///c:/Users/mseij/Desktop/PAROLC_Firmware/daemon/offsets.json).
- **Reset All Trims**: Restores all axes to 0.0° baseline trim.

---

## 8. Host Systemd Services & Deployment Automation

The control ecosystem is managed on Linux (Debian 13 Trixie / Raspberry Pi OS) via two dedicated systemd services:

1. **`parol6-daemon.service`** (or alias `parol6.service`):
   - **Path**: `/etc/systemd/system/parol6-daemon.service`
   - **Exec**: `/usr/bin/python3 /home/mark/parol6/daemon/robot_engine.py`
   - **Functions**: Manages USB serial connection to the ATmega2560 @ 250,000 baud, 100 Hz trajectory blending, kinematics engine, WebSocket server (`:8765`), and HTTP kiosk server (`:8000`).

2. **`parol6-kiosk.service`**:
   - **Path**: `/etc/systemd/system/parol6-kiosk.service`
   - **Exec**: `xinit /usr/local/bin/parol6-kiosk-browser.sh -- /usr/bin/Xorg :0 -s 0 -dpms -nocursor vt7`
   - **Functions**: Launches X11 and full-screen Chromium kiosk pointing to `http://127.0.0.1:8000`.

### 8.1 Automated Single-Command Deployment
- **Windows**: Run `Deploy-To-Pi.bat` or `powershell -ExecutionPolicy Bypass -File deploy\deploy_robot.ps1`.
  - Automatically compiles firmware using `arduino-cli`.
  - SCPs binaries and daemon files to the Pi (`mark@192.168.0.159`).
  - Halts services, flashes ATmega2560 via `avrdude` at 115,200 baud, verifies flash, and restarts services.
- **Linux / Pi**: Run `bash deploy/deploy_robot.sh`.

---

## 9. Hybrid UDP Transport Layer & High-Frequency Streaming (Port 8766)

To eliminate TCP Head-of-Line blocking and packet clumping during network jitter, the system implements a hybrid dual-transport architecture:
- **Port 8765 (TCP / WebSocket)**: Reliable JSON RPC for discrete commands, state mutations, homing, speed limits, and staging.
- **Port 8766 (Raw UDP)**: Unbuffered binary streaming for real-time motion (`MOVE_JOINTS`, `MOVE_CARTESIAN`, `HEARTBEAT`) and 50 Hz binary telemetry.

### 9.1 Packed 39-Byte Binary Frame Layout
Fixed-width, little-endian binary frame:
```
Offset   Field          Type        Description
0        Sync Header    uint8       0xAA (Constant sync frame delimiter)
1        Opcode         uint8       0x01 = Joints, 0x02 = Cartesian, 0x03 = Telemetry, 0x05 = Heartbeat
2..3     Seq            uint16_le   Monotonically incrementing packet counter
4..11    Timestamp_ns   uint64_le   Client monotonic timestamp in nanoseconds
12..35   Payload        6x float32  Joint angles in deg (Opcode 0x01) or Cartesian pose mm/deg (Opcode 0x02)
36       Gripper        uint8       Gripper setpoint (0–180)
37       Flags          uint8       Bit 0 = Continuous streaming active; Bit 1 = Cartesian vs Joint
38       CRC8           uint8       CRC-8-CCITT checksum byte (poly 0x07, init 0x00)
```

### 9.2 Host-Side Jitter Buffer (`daemon/robot_engine.py`)
- **Capacity**: 64-slot circular ring buffer (zero runtime allocations).
- **Target Horizon**: 60 ms nominal buffer depth.
- **Extraction Rate**: 100 Hz deterministic extraction loop feeding kinematics and step generator.
- **Anomaly Policies**: Out-of-order/late frames discarded, zero-velocity hold on underrun, leap-frog on overrun.

---

## 10. Automated Remote Hardware-In-The-Loop (HIL) Test Suite

The test suite (`daemon/tools/run_hardware_tests.py`) is executable locally or remotely over SSH (`deploy/test_remote.ps1`):
```powershell
powershell -ExecutionPolicy Bypass -File deploy\test_remote.ps1
```

### 10.1 Test Modules:
1. **Serial Clock Sync & Watchdog (`--test-serial`)**:
   - Sends 100 consecutive 'P' ping frames at 250k baud; asserts RTT <3.5 ms, 0 framing errors.
   - Stalls serial line for 300 ms; verifies Tier 2 safe state and status frame recovery.
2. **Kinematic Verification & LM Solver Benchmark (`--test-kinematics`)**:
   - Evaluates 50 reachable Cartesian waypoints; asserts position error <0.10 mm and average solve latency <1.5 ms on Raspberry Pi 5.
3. **UDP Jitter Buffer Stress (`--test-streaming`)**:
   - Injects 1,000 packed 32-byte frames at 100 Hz with ±15 ms temporal jitter; asserts uniform 100 Hz extraction with 0 queue overflows.
4. **E-Stop & State Purge Latency (`--test-estop`)**:
   - Populates motion queue, fires unbuffered `ESTOP`, asserts ACK latency <5.0 ms, and verifies queue depth drops to 0.

Summary report is output to `/tmp/hil_test_results.json` and mirrored locally to `docs/hil_test_results.json`.

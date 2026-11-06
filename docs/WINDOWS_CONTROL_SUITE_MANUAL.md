# PAROL6 Windows Control Suite: Developer Manual & Real-Time Joint Streaming API

---

## 1. Executive Summary & Distributed Architecture

This specification document serves as the complete engineering reference manual for developing native Windows desktop applications (C# .NET 8 / WPF, WinUI 3, or C++ Qt) to control the **PAROL6 6-DOF Robot Arm**.

### 1.1 The Real-Time Joint Streaming Paradigm (NO G-CODE STREAMING)
Unlike traditional CNC machines or 3D printers, modern multi-axis robotics requires continuous Cartesian trajectory generation, obstacle avoidance, and dynamic inverse kinematics (IK). The PAROL6 system uses a **Direct Real-Time Joint Streaming Protocol**:

- **NO G-code Streaming**: The robot does not parse or execute G-code. Trajectories and jog commands are transmitted strictly via high-frequency, structured JSON/WebSocket joint stream packets (`SERVO_JOINTS` / `STREAM_JOINTS` / `STREAM_TRAJECTORY`).
- **Windows PC as the Master Planner**: The Windows Control Suite calculates all 6-DOF Inverse Kinematics, Cartesian velocity profiles, path smoothing, and acceleration curves. The PC streams target joint angles $[q_1, q_2, q_3, q_4, q_5, q_6]$ (plus optional velocities and accelerations) at 20 Hz – 50 Hz.
- **Raspberry Pi as the Real-Time Joint Follower**: The Raspberry Pi control daemon (`robot_daemon.py`) receives the joint stream, validates soft limits, computes step differences $\Delta s_j = \Delta q_j \times \text{STEPS\_PER\_DEG}_j$, generates quadratic step chunks, and streams them with look-ahead flow control to the hardware ring buffers.
- **ATmega2560 + RAMPS 1.4 as the Master Pulse Generator**: Hardware Timer 1 (16 MHz master clock) executes the microsecond-accurate step pulse queues with zero jitter across all 6 axes simultaneously.

### 1.2 Mandatory Homing Safety Interlock
> [!IMPORTANT]
> **The Robot Must Be Homed Before Following Joint Streams**:
> Both the ATmega2560 firmware and the Raspberry Pi daemon enforce a strict hardware safety interlock: **the robot arm will not accept joint positions and must be homed** before it can follow joint angles or trajectory packets sent through this pipeline.
> - If `SERVO_JOINTS`, `STREAM_TRAJECTORY`, or `MOVE_JOINTS` is sent while the arm is not homed (`homed == false`), the daemon rejects the command (`status: "REJECTED"`), and the MCU rejects any step chunks with error code `0x05` (`Arm Not Homed`).
> - The Windows application must first execute `HOME` (`cmd: "HOME", joint: "ALL"`) and verify `homed == true` via the live 20 Hz telemetry stream before starting any streaming motion loop.

```
┌────────────────────────────────────────────────────────┐
│             Windows Control Suite (.NET 8 / WPF)       │
│  - Operator GUI, 3D Digital Twin, Path Planner        │
│  - 6-DOF Analytical / Numerical Inverse Kinematics     │
│  - Trajectory Interpolation & Velocity Profiler        │
│  - Real-Time Joint Streamer (20 Hz - 50 Hz Periodic)   │
└───────────────────────────┬────────────────────────────┘
                            │ TCP Port 8765 (JSON Stream)
                            │ SERVO_JOINTS / STREAM_TRAJECTORY
                            ▼
┌────────────────────────────────────────────────────────┐
│            Raspberry Pi 5 (robot_daemon.py)            │
│  - Real-Time Joint Follower & Pulse Feeder             │
│  - Physical Joint Limit Clamping & Safety Interlocks   │
│  - Look-Ahead Step Compressor (Quadratic Chunks)       │
│  - 20 Hz Telemetry Broadcast Feed                      │
└───────────────────────────┬────────────────────────────┘
                            │ USB-UART @ 250,000 Baud
                            ▼
┌────────────────────────────────────────────────────────┐
│        Arduino Mega 2560 + RAMPS 1.4 (AVR Firmware)    │
│  - Timer 1 16 MHz Hardware Queue (Dynamic OCR1A)       │
│  - 6 Joint Ring Buffers (First Clock, Count, Int, Add) │
│  - Direct Register Pin Toggling (PORTx, DDRx)          │
│  - Timer 3 50 Hz Hardware PWM RC Servo (D11 & D4)      │
└────────────────────────────────────────────────────────┘
```

---

## 2. Transport Layer & Communication Protocol

### 2.1 Connection Details
- **Protocol**: Standard WebSocket (RFC 6455) over TCP.
- **Default Port**: `8765`
- **Default Endpoint**: `ws://<pi-ip-or-hostname>:8765` (e.g. `ws://192.168.0.159:8765`).
- **Data Encoding**: UTF-8 encoded JSON text frames.

### 2.2 Dual-Loop Communication Architecture
The communication between the Windows Control Suite and the daemon operates across two parallel asynchronous pipelines:
1. **Inbound Joint Streaming & Command Pipeline (Client -> Daemon)**:
   - High-rate continuous joint stream (`SERVO_JOINTS`) sent at **20 Hz – 50 Hz** ($\Delta t = 20\text{--}50\text{ ms}$).
   - Trajectory buffer upload (`STREAM_TRAJECTORY`) for multi-point pre-calculated paths.
   - Discrete management commands (`ESTOP`, `RESET`, `HOME`, `JOG_JOINT`, `SET_GRIPPER`).
2. **Outbound Streaming Telemetry Feed (Daemon -> Client)**:
   - Continuous broadcast stream transmitted every **50 milliseconds (20 Hz)**.
   - Contains live joint angles, FK Cartesian TCP pose, endstop states, buffer fullness, and hardware flags.

---

## 3. Exhaustive Command Dictionary (Client -> Daemon)

### 3.1 `SERVO_JOINTS` — Continuous Real-Time Joint Follower (20–50 Hz)
- **Description**: The core continuous streaming command. Transmits target joint positions for all 6 joints to be reached over the time window `time` (seconds). The daemon clamps to soft limits, converts joint deltas into hardware step chunks, and chains them seamlessly into the MCU execution queue.
- **Rate**: Recommended 20 Hz to 50 Hz ($0.05\text{ s}$ to $0.02\text{ s}$).
- **Parameters**:
  - `joints` *(array of 6 floats, required)*: Target absolute joint angles $[J1, J2, J3, J4, J5, J6]$ in degrees.
  - `time` *(float, optional)*: Time duration in seconds for this streaming interval (default: `0.05`).
  - `velocities` *(array of 6 floats, optional)*: Instantaneous target joint velocities in deg/s.
  - `accelerations` *(array of 6 floats, optional)*: Instantaneous target joint accelerations in deg/s$^2$.
- **Payload**:
  ```json
  {
    "cmd": "SERVO_JOINTS",
    "joints": [12.5, -78.2, 165.0, 15.3, -5.2, 180.0],
    "time": 0.05
  }
  ```
- **Daemon ACK**:
  ```json
  {
    "type": "ACK",
    "cmd": "SERVO_JOINTS",
    "status": "OK"
  }
  ```

---

### 3.2 `STREAM_TRAJECTORY` — Dense Multi-Point Trajectory Buffer
- **Description**: Uploads a pre-calculated sequence of joint waypoints (e.g. from a spline or toolpath planner computed on Windows). The daemon streams them sequentially into the ATmega2560 ring buffer with look-ahead flow control.
- **Parameters**:
  - `points` *(array of objects, required)*: List of waypoint objects, each containing `joints` $[q_1..q_6]$ and `time` $\Delta t$.
- **Payload**:
  ```json
  {
    "cmd": "STREAM_TRAJECTORY",
    "points": [
      { "joints": [0.0, -90.0, 180.0, 0.0, 0.0, 180.0], "time": 0.02 },
      { "joints": [0.5, -89.5, 180.2, 0.1, 0.0, 180.0], "time": 0.02 },
      { "joints": [1.0, -89.0, 180.5, 0.2, 0.1, 180.0], "time": 0.02 }
    ]
  }
  ```

---

### 3.3 `JOG_JOINT` — Manual Joint Jog Increment
- **Description**: Moves a single joint by a relative degree increment. Used for manual jogging from Windows UI buttons or hotkeys.
- **Parameters**:
  - `joint` *(integer, 1..6)*: Joint index to move.
  - `delta_deg` *(float)*: Relative angular displacement in degrees (+/-).
  - `speed` *(float, 0.05..1.0)*: Speed scaling factor (default: `0.5`).
- **Payload**:
  ```json
  {
    "cmd": "JOG_JOINT",
    "joint": 1,
    "delta_deg": 5.0,
    "speed": 0.5
  }
  ```

---

### 3.4 `MOVE_JOINTS` — Point-to-Point Coordinated Joint Move
- **Description**: Moves all 6 joints to target absolute angles using synchronized trapezoidal acceleration profiles.
- **Parameters**:
  - `joints` *(array of 6 floats)*: Target joint angles $[J1, J2, J3, J4, J5, J6]$ in degrees.
  - `speed` *(float, 0.05..1.0)*: Speed factor.
- **Payload**:
  ```json
  {
    "cmd": "MOVE_JOINTS",
    "joints": [0.0, -90.0, 180.0, 0.0, 0.0, 180.0],
    "speed": 0.5
  }
  ```

---

### 3.5 `HOME` — Hardware Endstop Homing Sequence
- **Description**: Triggers the non-blocking hardware homing state machine on the ATmega2560. Executes safe mechanical sequence: $\text{J2} \to \text{J3} \to \text{J1} \to \text{J4} \to \text{J6} \to \text{J5} \to \text{CAD Home}$.
- **Parameters**:
  - `joint` *(string)*: `"ALL"` (default), `"WRIST"` (axes J4, J5, J6), or single axis `"J1"` through `"J6"`.
- **Payload**:
  ```json
  {
    "cmd": "HOME",
    "joint": "ALL"
  }
  ```

---

### 3.6 `ESTOP` — Immediate Emergency Stop
- **Description**: Instantly halts all motion without deceleration. The MCU cuts drive current to all stepper motor coils and purges all ring buffers.
- **Priority**: Critical / Preemptive.
- **Payload**:
  ```json
  {
    "cmd": "ESTOP"
  }
  ```

---

### 3.7 `RESET` — Clear E-Stop & Re-Energize Drivers
- **Description**: Clears the E-Stop lockout state and re-energizes motor coils with holding torque.
- **Payload**:
  ```json
  {
    "cmd": "RESET"
  }
  ```

---

### 3.8 `SET_GRIPPER` — Gripper Jaw Actuation
- **Description**: Controls the RC Servo Gripper via distance in millimeters ($0.0\text{--}62.5\text{ mm}$) or angle ($0\text{--}180^\circ$).
- **Payload**:
  ```json
  {
    "cmd": "SET_GRIPPER",
    "mm": 45.0
  }
  ```

---

### 3.9 `PRESET` — Move to Named Pose
- **Description**: Moves the arm to a predefined named joint configuration (`"CAD_HOME"`, `"CANDLE"`, `"PARK"`, `"STANDBY"`).
- **Payload**:
  ```json
  {
    "cmd": "PRESET",
    "name": "CAD_HOME",
    "speed": 0.5
  }
  ```

---

### 3.10 `SET_SOFT_LIMITS` — Runtime Joint Boundary Protection
- **Description**: Configures dynamic angular limits $[min^\circ, max^\circ]$ for joints 1 through 6.
- **Payload**:
  ```json
  {
    "cmd": "SET_SOFT_LIMITS",
    "limits": [
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

### 3.11 `GET_TRIMS` — Query Persistent Homing Calibration Trims
- **Description**: Queries the current persistent joint angular offsets (trims) stored on the Raspberry Pi host.
- **Payload**:
  ```json
  {
    "cmd": "GET_TRIMS"
  }
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

---

### 3.12 `SET_TRIMS` — Update & Save Persistent Homing Calibration Trims
- **Description**: Updates one or more joint calibration trims in real time and optionally commits them to `/home/mark/parol6/daemon/offsets.json` on the Raspberry Pi. This enables fine-tuning physical switch placement variations without modifying or recompiling microcontroller firmware.
- **Payload**:
  ```json
  {
    "cmd": "SET_TRIMS",
    "trims": {
      "j1": 0.5,
      "j2": -0.25
    },
    "save": true
  }
  ```
- **Parameters**:
  - `trims` *(object, required)*: Key-value map of joint names (`"j1"` through `"j6"`) to trim offsets in degrees.
  - `save` *(boolean, optional, default: `true`)*: When `true`, persists changes directly into `offsets.json`.
- **Daemon ACK**:
  ```json
  {
    "type": "ACK",
    "cmd": "SET_TRIMS",
    "status": "OK",
    "homing_trims_deg": {
      "j1": 0.5,
      "j2": -0.25,
      "j3": 0.0,
      "j4": 0.0,
      "j5": 0.0,
      "j6": 0.0
    }
  }
  ```

---

## 4. Live Telemetry Stream Specification (Daemon -> Client)

Every 50 ms (20 Hz), the daemon broadcasts a full telemetry state frame to all connected clients:

```json
{
  "type": "TELEMETRY",
  "state": "IDLE",
  "mcu_connected": true,
  "mcu_port": "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
  "mcu_rx_ago_s": 0.02,
  "packets_rx": 14205,
  "packets_tx": 14210,
  "stream_packets_rx": 520,
  "streaming_active": true,
  "joints_deg": [0.0, -90.0, 180.0, 0.0, 0.0, 180.0],
  "target_joints_deg": [0.0, -90.0, 180.0, 0.0, 0.0, 180.0],
  "pose_mm_deg": [341.77, 0.0, 334.0, 0.0, -90.0, 0.0],
  "gripper_mm": 62.5,
  "homed": true,
  "endstops_mask": 0,
  "endstops": [false, false, false, false, false, false],
  "mcu_clock": 41829104,
  "rtt_ms": 0.48,
  "drift_ppm": 4.2,
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

### Key Field Descriptions:
- `state`: `"IDLE"` (ready), `"MOVING"` (executing steps), `"HOMING"` (homing active), `"ESTOP"` (safety stopped).
- `joints_deg`: Exact real-time calibrated joint angles derived directly from the ATmega2560 hardware step registers plus persistent calibration trim: $q_j = \frac{\text{step\_pos}_j}{\text{STEPS\_PER\_DEG}_j} + \text{trim}_j$.
- `pose_mm_deg`: Forward Kinematics TCP pose $[X, Y, Z, Rx, Ry, Rz]$ in millimeters and degrees.
- `streaming_active`: `true` if the daemon received a `SERVO_JOINTS` packet within the last 500 ms.
- `homed`: `true` when all axes have latched home switches and aligned to CAD reference.
- `free_slots`: Available chunk slots in each joint's 16-chunk hardware ring buffer.

---

## 5. Physical Constants & Step Conversion

When calculating step counts or angular bounds on Windows, use these exact hardware constants:

| Joint | Axis Function | Gear Ratio | Microstepping | Steps / Degree | Physical Soft Limits |
|:-----:|:-------------:|:----------:|:-------------:|:--------------:|:--------------------:|
| **J1**| Base Yaw      | 10:1       | 16x (3200 s/r)| **88.888889**  | $-175^\circ \dots +175^\circ$ |
| **J2**| Shoulder Pitch| 50:1       | 16x (3200 s/r)| **444.444444** | $-150^\circ \dots +50^\circ$  |
| **J3**| Elbow Pitch   | 50:1       | 16x (3200 s/r)| **444.444444** | $+65^\circ \dots +290^\circ$  |
| **J4**| Forearm Roll  | 805/180    | 16x (3200 s/r)| **39.753086**  | $-240^\circ \dots +180^\circ$ |
| **J5**| Wrist Pitch   | 480/109    | 16x (3200 s/r)| **39.143731**  | $-135^\circ \dots +150^\circ$ |
| **J6**| Wrist Roll    | 1:1        | 16x (3200 s/r)| **8.888889**   | $-180^\circ \dots +360^\circ$ |

### CAD Home Position:
$$\mathbf{q}_{\text{CAD}} = [J1 = 0^\circ,\; J2 = -90^\circ,\; J3 = 180^\circ,\; J4 = 0^\circ,\; J5 = 0^\circ,\; J6 = 180^\circ]$$

### Persistent Homing Offset Trims:
Physical limit switch placement differences and mechanical contact hysteresis are adjusted dynamically on the Raspberry Pi without reflashing firmware. Trims are stored in `/home/mark/parol6/daemon/offsets.json` and automatically applied to incoming joint positions:
$$\theta_j = \left(\frac{\text{steps}_j}{\text{STEPS\_PER\_DEG}_j}\right) + \text{trim}_j$$

---

## 6. Complete C# .NET 8 / WPF Reference Implementation

Below is the production-ready C# client library, trajectory streaming loop, and WPF ViewModel designed for high-performance joint streaming.

### 6.1 `Parol6RobotClient.cs` — Asynchronous Client Driver

```csharp
using System;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;

namespace Parol6.ControlSuite.Core
{
    public class TelemetryEventArgs : EventArgs
    {
        public string State { get; set; } = "UNKNOWN";
        public double[] JointsDeg { get; set; } = new double[6];
        public double[] TargetJointsDeg { get; set; } = new double[6];
        public double[] PoseMmDeg { get; set; } = new double[6];
        public double GripperMm { get; set; }
        public bool IsHomed { get; set; }
        public bool IsStreaming { get; set; }
        public bool[] Endstops { get; set; } = new bool[6];
        public double RttMs { get; set; }
        public uint McuClock { get; set; }
        public int[] BufferFree { get; set; } = new int[6];
    }

    public class Parol6RobotClient : IDisposable
    {
        private ClientWebSocket? _ws;
        private CancellationTokenSource? _cts;
        private Task? _receiveTask;

        public event EventHandler<TelemetryEventArgs>? TelemetryReceived;
        public event EventHandler<string>? StateChanged;
        public event EventHandler<bool>? ConnectionStatusChanged;

        public bool IsConnected => _ws?.State == WebSocketState.Open;
        public string CurrentState { get; private set; } = "DISCONNECTED";

        public async Task ConnectAsync(string hostIp, int port = 8765)
        {
            Disconnect();

            _cts = new CancellationTokenSource();
            _ws = new ClientWebSocket();
            _ws.Options.KeepAliveInterval = TimeSpan.FromSeconds(5);

            var uri = new Uri($"ws://{hostIp}:{port}");
            await _ws.ConnectAsync(uri, _cts.Token);

            ConnectionStatusChanged?.Invoke(this, true);
            _receiveTask = Task.Run(ReceiveLoopAsync, _cts.Token);
        }

        public void Disconnect()
        {
            try
            {
                _cts?.Cancel();
                if (_ws != null && _ws.State == WebSocketState.Open)
                {
                    _ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "Closing", CancellationToken.None).Wait(1000);
                }
            }
            catch { }
            finally
            {
                _ws?.Dispose();
                _ws = null;
                _cts?.Dispose();
                _cts = null;
                ConnectionStatusChanged?.Invoke(this, false);
            }
        }

        // ====================================================================
        // REAL-TIME JOINT STREAMING METHODS
        // ====================================================================

        /// <summary>
        /// Continuously stream real-time joint positions calculated by Windows IK (20-50 Hz).
        /// </summary>
        public async Task ServoJointsAsync(double[] jointAnglesDeg, double dtSeconds = 0.05, double[]? velocities = null, double[]? accelerations = null)
        {
            if (jointAnglesDeg.Length != 6) throw new ArgumentException("Must supply 6 joint angles", nameof(jointAnglesDeg));

            var payload = new
            {
                cmd = "SERVO_JOINTS",
                joints = jointAnglesDeg,
                time = dtSeconds,
                velocities = velocities,
                accelerations = accelerations
            };

            await SendCommandAsync(payload);
        }

        /// <summary>
        /// Upload a pre-computed multi-point trajectory to execute on the robot.
        /// </summary>
        public async Task StreamTrajectoryAsync(object trajectoryPoints)
        {
            await SendCommandAsync(new
            {
                cmd = "STREAM_TRAJECTORY",
                points = trajectoryPoints
            });
        }

        // ====================================================================
        // DISCRETE MANAGEMENT COMMANDS
        // ====================================================================

        public async Task EmergencyStopAsync()
        {
            await SendCommandAsync(new { cmd = "ESTOP" });
        }

        public async Task ResetAsync()
        {
            await SendCommandAsync(new { cmd = "RESET" });
        }

        public async Task HomeAsync(string joint = "ALL")
        {
            await SendCommandAsync(new { cmd = "HOME", joint = joint });
        }

        public async Task JogJointAsync(int jointIndex, double deltaDegrees, double speed = 0.5)
        {
            if (jointIndex < 1 || jointIndex > 6) throw new ArgumentOutOfRangeException(nameof(jointIndex));
            await SendCommandAsync(new
            {
                cmd = "JOG_JOINT",
                joint = jointIndex,
                delta_deg = deltaDegrees,
                speed = Math.Clamp(speed, 0.05, 1.0)
            });
        }

        public async Task MoveJointsAsync(double[] jointAnglesDeg, double speed = 0.5)
        {
            if (jointAnglesDeg.Length != 6) throw new ArgumentException("Must supply 6 joint angles", nameof(jointAnglesDeg));
            await SendCommandAsync(new
            {
                cmd = "MOVE_JOINTS",
                joints = jointAnglesDeg,
                speed = Math.Clamp(speed, 0.05, 1.0)
            });
        }

        public async Task SetGripperMmAsync(double gapMm)
        {
            await SendCommandAsync(new
            {
                cmd = "SET_GRIPPER",
                mm = Math.Clamp(gapMm, 0.0, 62.5)
            });
        }

        public async Task GoPresetAsync(string presetName, double speed = 0.5)
        {
            await SendCommandAsync(new
            {
                cmd = "PRESET",
                name = presetName.ToUpper(),
                speed = Math.Clamp(speed, 0.05, 1.0)
            });
        }

        // ====================================================================
        // TRANSPORT IMPLEMENTATION
        // ====================================================================

        private async Task SendCommandAsync(object payload)
        {
            if (_ws == null || _ws.State != WebSocketState.Open) return;

            string json = JsonSerializer.Serialize(payload);
            byte[] bytes = Encoding.UTF8.GetBytes(json);

            await _ws.SendAsync(
                new ArraySegment<byte>(bytes),
                WebSocketMessageType.Text,
                true,
                _cts?.Token ?? CancellationToken.None
            );
        }

        private async Task ReceiveLoopAsync()
        {
            var buffer = new byte[16384];
            while (_cts != null && !_cts.IsCancellationRequested && _ws != null && _ws.State == WebSocketState.Open)
            {
                try
                {
                    var result = await _ws.ReceiveAsync(new ArraySegment<byte>(buffer), _cts.Token);
                    if (result.MessageType == WebSocketMessageType.Close) break;

                    string jsonText = Encoding.UTF8.GetString(buffer, 0, result.Count);
                    using var doc = JsonDocument.Parse(jsonText);
                    var root = doc.RootElement;

                    if (root.TryGetProperty("type", out var typeProp) && typeProp.GetString() == "TELEMETRY")
                    {
                        var telem = new TelemetryEventArgs();
                        telem.State = root.GetProperty("state").GetString() ?? "UNKNOWN";
                        telem.GripperMm = root.GetProperty("gripper_mm").GetDouble();
                        telem.IsHomed = root.GetProperty("homed").GetBoolean();
                        telem.RttMs = root.GetProperty("rtt_ms").GetDouble();
                        telem.McuClock = root.GetProperty("mcu_clock").GetUInt32();
                        telem.IsStreaming = root.TryGetProperty("streaming_active", out var sProp) && sProp.GetBoolean();

                        int idx = 0;
                        foreach (var el in root.GetProperty("joints_deg").EnumerateArray())
                        {
                            if (idx < 6) telem.JointsDeg[idx++] = el.GetDouble();
                        }

                        idx = 0;
                        if (root.TryGetProperty("target_joints_deg", out var tgtProp))
                        {
                            foreach (var el in tgtProp.EnumerateArray())
                            {
                                if (idx < 6) telem.TargetJointsDeg[idx++] = el.GetDouble();
                            }
                        }

                        idx = 0;
                        foreach (var el in root.GetProperty("pose_mm_deg").EnumerateArray())
                        {
                            if (idx < 6) telem.PoseMmDeg[idx++] = el.GetDouble();
                        }

                        idx = 0;
                        foreach (var el in root.GetProperty("endstops").EnumerateArray())
                        {
                            if (idx < 6) telem.Endstops[idx++] = el.GetBoolean();
                        }

                        idx = 0;
                        foreach (var el in root.GetProperty("free_slots").EnumerateArray())
                        {
                            if (idx < 6) telem.BufferFree[idx++] = el.GetInt32();
                        }

                        if (telem.State != CurrentState)
                        {
                            CurrentState = telem.State;
                            StateChanged?.Invoke(this, CurrentState);
                        }

                        TelemetryReceived?.Invoke(this, telem);
                    }
                }
                catch (OperationCanceledException) { break; }
                catch { break; }
            }
            ConnectionStatusChanged?.Invoke(this, false);
        }

        public void Dispose()
        {
            Disconnect();
        }
    }
}
```

---

### 6.2 `JointTrajectoryStreamer.cs` — 20–50 Hz Real-Time Streaming Loop

This class runs a precision timer loop on Windows that calculates the next joint target from your Cartesian trajectory planner / IK solver and streams it to the robot:

```csharp
using System;
using System.Threading;
using System.Threading.Tasks;

namespace Parol6.ControlSuite.Core
{
    public class JointTrajectoryStreamer
    {
        private readonly Parol6RobotClient _client;
        private CancellationTokenSource? _streamCts;
        private Task? _streamTask;

        public bool IsStreaming => _streamTask != null && !_streamTask.IsCompleted;

        public JointTrajectoryStreamer(Parol6RobotClient client)
        {
            _client = client;
        }

        /// <summary>
        /// Start streaming real-time joint positions at the specified frequency (e.g. 20 Hz to 50 Hz).
        /// </summary>
        /// <param name="jointTargetGenerator">Function returning current 6 joint angles [deg] at time t.</param>
        /// <param name="frequencyHz">Streaming rate in Hz (default: 20 Hz, interval: 50 ms).</param>
        public void StartStreaming(Func<double, double[]> jointTargetGenerator, int frequencyHz = 20)
        {
            StopStreaming();

            _streamCts = new CancellationTokenSource();
            double dt = 1.0 / frequencyHz;
            var interval = TimeSpan.FromMilliseconds(dt * 1000.0);

            _streamTask = Task.Run(async () =>
            {
                using var timer = new PeriodicTimer(interval);
                var stopwatch = System.Diagnostics.Stopwatch.StartNew();

                while (!_streamCts.Token.IsCancellationRequested && await timer.WaitForNextTickAsync(_streamCts.Token))
                {
                    double t = stopwatch.Elapsed.TotalSeconds;
                    double[] targetJoints = jointTargetGenerator(t);

                    if (_client.IsConnected)
                    {
                        await _client.ServoJointsAsync(targetJoints, dtSeconds: dt);
                    }
                }
            }, _streamCts.Token);
        }

        public void StopStreaming()
        {
            try
            {
                _streamCts?.Cancel();
                _streamTask?.Wait(500);
            }
            catch { }
            finally
            {
                _streamCts?.Dispose();
                _streamCts = null;
                _streamTask = null;
            }
        }
    }
}
```

---

### 6.3 `MainViewModel.cs` — WPF MVVM Controller

```csharp
using System;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows;
using Parol6.ControlSuite.Core;

namespace Parol6.ControlSuite.ViewModels
{
    public class MainViewModel : INotifyPropertyChanged
    {
        private readonly Parol6RobotClient _client = new();
        private readonly JointTrajectoryStreamer _streamer;

        private string _targetIp = "192.168.0.159";
        private string _robotState = "DISCONNECTED";
        private bool _isConnected;
        private double _jointJogStep = 5.0;
        private double _moveSpeed = 0.5;

        // Joint angles (Live from MCU registers)
        private double[] _joints = new double[6];
        private double[] _pose = new double[6];
        private double _gripperGap = 62.5;
        private double _rttMs;
        private bool _isStreamingActive;

        public MainViewModel()
        {
            _streamer = new JointTrajectoryStreamer(_client);

            _client.ConnectionStatusChanged += (s, connected) =>
            {
                IsConnected = connected;
                OnPropertyChanged(nameof(IsConnected));
            };

            _client.TelemetryReceived += (s, e) =>
            {
                Application.Current.Dispatcher.Invoke(() =>
                {
                    RobotState = e.State;
                    RttMs = e.RttMs;
                    GripperGap = e.GripperMm;
                    IsStreamingActive = e.IsStreaming;

                    for (int i = 0; i < 6; i++)
                    {
                        _joints[i] = e.JointsDeg[i];
                        _pose[i] = e.PoseMmDeg[i];
                    }

                    OnPropertyChanged(nameof(J1));
                    OnPropertyChanged(nameof(J2));
                    OnPropertyChanged(nameof(J3));
                    OnPropertyChanged(nameof(J4));
                    OnPropertyChanged(nameof(J5));
                    OnPropertyChanged(nameof(J6));
                    OnPropertyChanged(nameof(PosX));
                    OnPropertyChanged(nameof(PosY));
                    OnPropertyChanged(nameof(PosZ));
                    OnPropertyChanged(nameof(PosRx));
                    OnPropertyChanged(nameof(PosRy));
                    OnPropertyChanged(nameof(PosRz));
                });
            };
        }

        // --- PROPERTIES ---
        public string TargetIp { get => _targetIp; set { _targetIp = value; OnPropertyChanged(); } }
        public string RobotState { get => _robotState; set { _robotState = value; OnPropertyChanged(); } }
        public bool IsConnected { get => _isConnected; set { _isConnected = value; OnPropertyChanged(); } }
        public bool IsStreamingActive { get => _isStreamingActive; set { _isStreamingActive = value; OnPropertyChanged(); } }
        public double RttMs { get => _rttMs; set { _rttMs = value; OnPropertyChanged(); } }
        public double GripperGap { get => _gripperGap; set { _gripperGap = value; OnPropertyChanged(); } }

        public double J1 => _joints[0];
        public double J2 => _joints[1];
        public double J3 => _joints[2];
        public double J4 => _joints[3];
        public double J5 => _joints[4];
        public double J6 => _joints[5];

        public double PosX => _pose[0];
        public double PosY => _pose[1];
        public double PosZ => _pose[2];
        public double PosRx => _pose[3];
        public double PosRy => _pose[4];
        public double PosRz => _pose[5];

        // --- COMMAND METHODS ---
        public async void Connect() => await _client.ConnectAsync(TargetIp);
        public void Disconnect() => _client.Disconnect();
        public async void EmergencyStop() => await _client.EmergencyStopAsync();
        public async void ResetMotors() => await _client.ResetAsync();
        public async void HomeAll() => await _client.HomeAsync("ALL");

        public async void JogJoint(int joint, double direction)
        {
            await _client.JogJointAsync(joint, direction * _jointJogStep, _moveSpeed);
        }

        public async void SetGripper(double mm)
        {
            await _client.SetGripperMmAsync(mm);
        }

        public event PropertyChangedEventHandler? PropertyChanged;
        protected void OnPropertyChanged([CallerMemberName] string? name = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
        }
    }
}
```

---

## 7. Zero-Delay Hardware Emergency Stop Guidelines for Windows

When interfacing physical robotics with desktop GUI frameworks (WPF / WinUI):
1. **Never bind Emergency Stop to `Click` or `MouseUp`**: Standard Windows UI button controls incur up to a 100 ms release delay and can be blocked if modal message boxes appear.
2. **Bind to `PreviewMouseDown` / `PreviewTouchDown`**: Attach the E-Stop handler directly to the tunnel event:
   ```csharp
   btnEmergencyStop.PreviewMouseDown += (s, e) => {
       _ = client.EmergencyStopAsync();
   };
   ```
3. **Register Global Hotkey**: Register a system-wide hotkey (e.g. `Spacebar` or `Esc`) via `RegisterHotKey(hWnd, ...)` so that hitting the key instantly sends `ESTOP` even if the window is in the background.

---

## 8. Summary of Architectural Guarantees

| Requirement | Implementation Guarantee |
|:---|:---|
| **No G-code Streaming** | Communicates strictly via structured JSON frames (`SERVO_JOINTS`). Zero G-code parsing on RPi. |
| **Inverse Kinematics** | Calculated on the Windows Control Suite PC; RPi operates as a pure real-time joint follower. |
| **Look-Ahead Jitter Buffer** | RPi chains quadratic step chunks (`first_step_clock, count, interval, add, dir`) into 16-chunk hardware buffers on ATmega2560. |
| **Real-Time Follower** | Streams at 20 Hz to 50 Hz with continuous phase-locked step execution. |
| **Safety Interlocks** | Hardware E-Stop pin disconnects coils immediately; RPi clamps target joint angles against software boundaries. |

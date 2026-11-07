using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Threading;
using System.Threading.Tasks;

namespace PAROLC_ControlSuite.Network
{
    public class CommLogEntry
    {
        public int Id { get; set; }
        public string Ts { get; set; } = "";
        public string Link { get; set; } = ""; // "SUITE_RPI" or "RPI_MCU"
        public string Dir { get; set; } = "";  // "RX", "TX", or "ERR"
        public string Msg { get; set; } = "";
    }

    public class TelemetryData
    {
        public string State { get; set; } = "DISCONNECTED";
        public double[] JointsDeg { get; set; } = new double[6];
        public double[] TargetJointsDeg { get; set; } = new double[6];
        public double[] PoseMmDeg { get; set; } = new double[6]; // [X, Y, Z, Roll, Pitch, Yaw]
        public double GripperMm { get; set; } = 0.0;
        public bool Homed { get; set; } = false;
        public int HomedMask { get; set; } = 0;
        public int EndstopsMask { get; set; } = 0;
        public bool[] Endstops { get; set; } = new bool[6];
        public uint McuClock { get; set; } = 0;
        public double RttMs { get; set; } = 0.0;
        public double DriftPpm { get; set; } = 0.0;
        public int[] FreeSlots { get; set; } = new int[6];
        public int DirInvertMask { get; set; } = 3;
        public int EndstopInvertMask { get; set; } = 0;
        public bool McuConnected { get; set; } = false;
        public bool KlipperConnected { get; set; } = false;
        public bool KlipperReady { get; set; } = false;
        public double SpeedScale { get; set; } = 0.4;
        public bool StreamingActive { get; set; } = false;
        public string Warning { get; set; } = "";
        public string LastMcuError { get; set; } = "";
        public List<CommLogEntry> CommLogs { get; set; } = new List<CommLogEntry>();
        public Dictionary<string, double> HomingTrimsDeg { get; set; } = new Dictionary<string, double>();
    }

    public enum ConnectionState
    {
        Disconnected,
        Connecting,
        Connected,
        Reconnecting,
        Error
    }

    public class RobotWebSocketClient : IDisposable
    {
        // CRC-8-CCITT lookup table (Polynomial 0x07, Initial value 0x00)
        private static readonly byte[] Crc8Table = new byte[256];

        static RobotWebSocketClient()
        {
            byte poly = 0x07;
            for (int i = 0; i < 256; i++)
            {
                byte curr = (byte)i;
                for (int b = 0; b < 8; b++)
                {
                    if ((curr & 0x80) != 0)
                        curr = (byte)(((curr << 1) ^ poly) & 0xFF);
                    else
                        curr = (byte)((curr << 1) & 0xFF);
                }
                Crc8Table[i] = curr;
            }
        }

        public static byte ComputeCrc8(ReadOnlySpan<byte> data)
        {
            byte val = 0;
            for (int i = 0; i < data.Length; i++)
            {
                val = Crc8Table[val ^ data[i]];
            }
            return val;
        }

        public static ulong GetCurrentTimestampNs()
        {
            return (ulong)(Stopwatch.GetTimestamp() * (1_000_000_000.0 / Stopwatch.Frequency));
        }

        private ClientWebSocket _ws;
        private CancellationTokenSource _cts;
        private Task _receiveLoopTask;
        private Task _pingLoopTask;
        private readonly SemaphoreSlim _sendLock = new SemaphoreSlim(1, 1);

        // Raw UDP Motion Streaming (Port 8766)
        private UdpClient _udpClient;
        private IPEndPoint _udpRemoteEndPoint;
        private Task _udpReceiveLoopTask;

        public string HostAddress { get; set; } = "192.168.0.159:8765";
        public ConnectionState State { get; private set; } = ConnectionState.Disconnected;
        public double MeasuredRttMs { get; private set; } = 0.0;
        public double UpdateRateHz { get; private set; } = 0.0;

        // UDP Real-Time Network Metrics
        public long UdpPacketsTx { get; private set; } = 0;
        public long UdpPacketsRx { get; private set; } = 0;
        public double UdpRttMs { get; private set; } = 0.0;
        public double UdpJitterMs { get; private set; } = 0.0;
        public double UdpPacketLossPercent { get; private set; } = 0.0;
        public double UdpTxRateHz { get; private set; } = 0.0;
        public double UdpRxRateHz { get; private set; } = 0.0;

        // Events
        public event Action<ConnectionState> StateChanged;
        public event Action<TelemetryData> TelemetryReceived;
        public event Action<string> LogMessage;
        public event Action<Dictionary<string, double>> TrimsReceived;
        public event Action<string, string> RawPacketLogged; // direction ("TX" | "RX"), payload json
        public event Action<double, double, double, double> UdpMetricsUpdated; // (rttMs, jitterMs, lossPercent, rxRateHz)

        public long TotalPacketsRx { get; private set; } = 0;
        public long TotalPacketsTx { get; private set; } = 0;

        // Telemetry rate calculator
        private int _frameCounter = 0;
        private readonly Stopwatch _rateStopwatch = new Stopwatch();
        private readonly Stopwatch _heartbeatStopwatch = new Stopwatch();
        private ushort _sendSeq = 0;

        public RobotWebSocketClient()
        {
            _rateStopwatch.Start();
        }

        public async Task ConnectAsync(string address = null)
        {
            if (address != null) HostAddress = address;

            Disconnect();

            _cts = new CancellationTokenSource();
            _ws = new ClientWebSocket();
            _ws.Options.KeepAliveInterval = TimeSpan.FromSeconds(5);

            string rawAddr = (HostAddress ?? "").Trim();
            if (rawAddr.StartsWith("ws://", StringComparison.OrdinalIgnoreCase))
                rawAddr = rawAddr.Substring(5);
            else if (rawAddr.StartsWith("http://", StringComparison.OrdinalIgnoreCase))
                rawAddr = rawAddr.Substring(7);

            if (!rawAddr.Contains(":"))
                rawAddr += ":8765";

            string uriStr = $"ws://{rawAddr}";

            SetState(ConnectionState.Connecting);
            LogMessage?.Invoke($"Connecting to {uriStr} (TCP WebSocket) & UDP 8766...");

            try
            {
                var uri = new Uri(uriStr);
                await _ws.ConnectAsync(uri, _cts.Token);

                // Initialize UDP Streaming Socket targeting port 8766
                string hostIp = uri.Host;
                try
                {
                    _udpClient = new UdpClient();
                    _udpRemoteEndPoint = new IPEndPoint(IPAddress.Parse(hostIp), 8766);
                    _udpClient.Connect(_udpRemoteEndPoint);
                    _udpReceiveLoopTask = Task.Run(UdpReceiveLoopAsync, _cts.Token);
                    SendUdpHeartbeat();
                    LogMessage?.Invoke($"Raw UDP 6-Axis motion streaming channel active: {hostIp}:8766");
                }
                catch (Exception udpEx)
                {
                    LogMessage?.Invoke($"UDP initialization warning: {udpEx.Message}");
                }

                SetState(ConnectionState.Connected);
                LogMessage?.Invoke("Connected to PAROL6 6-Axis Klipper Stack.");

                _receiveLoopTask = Task.Run(ReceiveLoopAsync, _cts.Token);
                _pingLoopTask = Task.Run(PingLoopAsync, _cts.Token);

                // Request initial persistent trims
                await SendGetTrimsAsync();
            }
            catch (Exception ex)
            {
                SetState(ConnectionState.Error);
                LogMessage?.Invoke($"Connection failed: {ex.Message}");
            }
        }

        public void Disconnect()
        {
            try
            {
                _cts?.Cancel();
                if (_ws != null && _ws.State == WebSocketState.Open)
                {
                    _ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "Closing", CancellationToken.None).Wait(500);
                }
                _udpClient?.Close();
                _udpClient?.Dispose();
                _udpClient = null;
            }
            catch { }
            finally
            {
                _ws?.Dispose();
                _ws = null;
                SetState(ConnectionState.Disconnected);
            }
        }

        private void SetState(ConnectionState newState)
        {
            if (State != newState)
            {
                State = newState;
                StateChanged?.Invoke(State);
            }
        }

        private async Task ReceiveLoopAsync()
        {
            var buffer = new byte[8192];
            var ms = new System.IO.MemoryStream();

            while (_cts != null && !_cts.IsCancellationRequested && _ws != null && _ws.State == WebSocketState.Open)
            {
                try
                {
                    ms.SetLength(0);
                    WebSocketReceiveResult result;
                    do
                    {
                        result = await _ws.ReceiveAsync(new ArraySegment<byte>(buffer), _cts.Token);
                        if (result.MessageType == WebSocketMessageType.Close)
                        {
                            SetState(ConnectionState.Disconnected);
                            return;
                        }
                        ms.Write(buffer, 0, result.Count);
                    } while (!result.EndOfMessage);

                    if (result.MessageType == WebSocketMessageType.Binary)
                    {
                        byte[] bytes = ms.ToArray();
                        TotalPacketsRx++;
                        ProcessBinaryMessage(bytes);
                    }
                    else
                    {
                        string jsonText = Encoding.UTF8.GetString(ms.ToArray());
                        TotalPacketsRx++;
                        RawPacketLogged?.Invoke("RX", jsonText);
                        ProcessMessage(jsonText);
                    }

                    // Track telemetry frequency
                    _frameCounter++;
                    if (_rateStopwatch.ElapsedMilliseconds >= 1000)
                    {
                        UpdateRateHz = _frameCounter * 1000.0 / _rateStopwatch.ElapsedMilliseconds;
                        _frameCounter = 0;
                        _rateStopwatch.Restart();
                    }
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                catch (Exception ex)
                {
                    LogMessage?.Invoke($"Receive error: {ex.Message}");
                    break;
                }
            }

            SetState(ConnectionState.Disconnected);
        }

        private void ProcessBinaryMessage(byte[] bytes)
        {
            if (bytes == null || bytes.Length != 31 || bytes[0] != 0xAA) return;

            // Verify XOR CRC8
            byte crc = 0;
            for (int i = 0; i < 30; i++) crc ^= bytes[i];
            if (crc != bytes[30]) return;

            byte opcode = bytes[1];
            ushort seq = BitConverter.ToUInt16(bytes, 2);

            if (opcode == 0x03) // Opcode 0x03: High-Speed Telemetry Frame
            {
                float j1 = BitConverter.ToSingle(bytes, 4);
                float j2 = BitConverter.ToSingle(bytes, 8);
                float j3 = BitConverter.ToSingle(bytes, 12);
                float j4 = BitConverter.ToSingle(bytes, 16);
                float j5 = BitConverter.ToSingle(bytes, 20);
                float j6 = BitConverter.ToSingle(bytes, 24);
                byte gripper = bytes[28];
                byte flags = bytes[29];

                bool estop = (flags & 0x01) != 0;
                bool homing = (flags & 0x02) != 0;
                bool moving = (flags & 0x04) != 0;
                bool homed = (flags & 0x08) != 0;

                string state = estop ? "ESTOP" : homing ? "HOMING" : moving ? "STREAMING" : "IDLE";

                var telem = new TelemetryData
                {
                    State = state,
                    Homed = homed,
                    HomedMask = homed ? 0x3F : 0x00,
                    JointsDeg = new double[] { j1, j2, j3, j4, j5, j6 },
                    GripperMm = Math.Round(gripper / 2.55, 1),
                    KlipperConnected = true,
                    KlipperReady = true,
                    RttMs = MeasuredRttMs
                };
                TelemetryReceived?.Invoke(telem);
            }
            else if (opcode == 0x05) // Opcode 0x05: Heartbeat Pong
            {
                MeasuredRttMs = Math.Round(_heartbeatStopwatch.Elapsed.TotalMilliseconds, 2);
            }
        }

        private void ProcessMessage(string jsonText)
        {
            try
            {
                var doc = JsonNode.Parse(jsonText);
                if (doc == null) return;

                string type = (string)doc["type"] ?? "";

                if (type == "TELEMETRY")
                {
                    var telem = new TelemetryData
                    {
                        State = (string)doc["state"] ?? "IDLE",
                        Homed = (bool)(doc["homed"] ?? false),
                        HomedMask = (int)(doc["homed_mask"] ?? 0),
                        EndstopsMask = (int)(doc["endstops_mask"] ?? 0),
                        McuClock = (uint)(doc["mcu_clock"] ?? 0),
                        RttMs = (double)(doc["rtt_ms"] ?? 0.0),
                        DriftPpm = (double)(doc["drift_ppm"] ?? 0.0),
                        GripperMm = (double)(doc["gripper_mm"] ?? 0.0),
                        McuConnected = (bool)(doc["mcu_connected"] ?? doc["klipper_connected"] ?? false),
                        KlipperConnected = (bool)(doc["klipper_connected"] ?? false),
                        KlipperReady = (bool)(doc["klipper_ready"] ?? false),
                        SpeedScale = (double)(doc["speed_scale"] ?? 0.4),
                        StreamingActive = (bool)(doc["streaming_active"] ?? false),
                        DirInvertMask = (int)(doc["dir_invert_mask"] ?? 3),
                        EndstopInvertMask = (int)(doc["endstop_invert_mask"] ?? 0)
                    };

                    // Joint angles [J1..J6]
                    if (doc["joints_deg"] is JsonArray jArr && jArr.Count >= 6)
                    {
                        for (int i = 0; i < 6; i++) telem.JointsDeg[i] = (double)jArr[i]!;
                    }

                    // Target Joint angles [J1..J6]
                    if (doc["target_joints_deg"] is JsonArray tjArr && tjArr.Count >= 6)
                    {
                        for (int i = 0; i < 6; i++) telem.TargetJointsDeg[i] = (double)tjArr[i]!;
                    }

                    // Pose: bind [x, y, z, roll, pitch, yaw]
                    if (doc["pose_mm_deg"] is JsonArray pArr && pArr.Count >= 6)
                    {
                        for (int i = 0; i < 6; i++) telem.PoseMmDeg[i] = (double)pArr[i]!;
                    }
                    else if (doc["pose"] is JsonObject pObj)
                    {
                        telem.PoseMmDeg[0] = (double)(pObj["x"] ?? 0.0);
                        telem.PoseMmDeg[1] = (double)(pObj["y"] ?? 0.0);
                        telem.PoseMmDeg[2] = (double)(pObj["z"] ?? 0.0);
                        telem.PoseMmDeg[3] = (double)(pObj["roll"] ?? pObj["rx"] ?? 0.0);
                        telem.PoseMmDeg[4] = (double)(pObj["pitch"] ?? pObj["ry"] ?? 0.0);
                        telem.PoseMmDeg[5] = (double)(pObj["yaw"] ?? pObj["rz"] ?? 0.0);
                    }

                    // Safety warnings (e.g. Branch Jump Guard)
                    if (doc["warning"] != null)
                    {
                        telem.Warning = (string)doc["warning"] ?? "";
                    }

                    if (doc["last_mcu_error"] != null)
                    {
                        telem.LastMcuError = (string)doc["last_mcu_error"] ?? "";
                    }

                    if (doc["comm_logs"] is JsonArray cLogs)
                    {
                        foreach (var item in cLogs)
                        {
                            if (item is JsonObject lObj)
                            {
                                telem.CommLogs.Add(new CommLogEntry
                                {
                                    Id = (int)(lObj["id"] ?? 0),
                                    Ts = (string)lObj["ts"] ?? "",
                                    Link = (string)lObj["link"] ?? "",
                                    Dir = (string)lObj["dir"] ?? "",
                                    Msg = (string)lObj["msg"] ?? ""
                                });
                            }
                        }
                    }

                    // Endstops
                    if (doc["endstops_list"] is JsonArray elArr && elArr.Count >= 6)
                    {
                        for (int i = 0; i < 6; i++) telem.Endstops[i] = (bool)elArr[i]!;
                    }
                    else if (doc["endstops"] is JsonArray eArr && eArr.Count >= 6)
                    {
                        for (int i = 0; i < 6; i++) telem.Endstops[i] = (bool)eArr[i]!;
                    }
                    else if (doc["endstops"] is JsonObject eObj)
                    {
                        for (int i = 0; i < 6; i++)
                        {
                            string key = $"j{i + 1}";
                            telem.Endstops[i] = (bool)(eObj[key] ?? false);
                        }
                    }
                    else
                    {
                        for (int i = 0; i < 6; i++)
                        {
                            telem.Endstops[i] = (telem.EndstopsMask & (1 << i)) != 0;
                        }
                    }

                    TelemetryReceived?.Invoke(telem);
                }
                else if (type == "ACK")
                {
                    string status = (string)doc["status"] ?? "OK";
                    string cmd = (string)doc["cmd"] ?? "";
                    string msg = (string)doc["msg"] ?? "";
                    if (status != "OK" || !string.IsNullOrEmpty(msg))
                    {
                        LogMessage?.Invoke($"[{cmd}] {status}: {msg}");
                    }
                }
                else if (type == "TRIMS" || (doc["homing_trims_deg"] != null))
                {
                    var trimsNode = doc["homing_trims_deg"];
                    if (trimsNode is JsonObject obj)
                    {
                        var dict = new Dictionary<string, double>();
                        foreach (var kvp in obj)
                        {
                            dict[kvp.Key] = (double)kvp.Value!;
                        }
                        TrimsReceived?.Invoke(dict);
                    }
                }
            }
            catch (Exception ex)
            {
                LogMessage?.Invoke($"JSON Parse Error: {ex.Message}");
            }
        }

        private async Task PingLoopAsync()
        {
            while (_cts != null && !_cts.IsCancellationRequested && _ws != null && _ws.State == WebSocketState.Open)
            {
                try
                {
                    await Task.Delay(66, _cts.Token); // ~15 Hz continuous heartbeat ping
                    await SendBinaryHeartbeatAsync();
                }
                catch (OperationCanceledException) { break; }
                catch { }
            }
        }

        public async Task SendBinaryHeartbeatAsync()
        {
            if (_ws == null || _ws.State != WebSocketState.Open) return;

            byte[] packet = new byte[31];
            packet[0] = 0xAA;
            packet[1] = 0x05; // Opcode 0x05: Heartbeat
            BitConverter.TryWriteBytes(packet.AsSpan(2, 2), (ushort)(_sendSeq++ & 0xFFFF));
            packet[28] = 0;
            packet[29] = 0;

            byte crc = 0;
            for (int i = 0; i < 30; i++) crc ^= packet[i];
            packet[30] = crc;

            _heartbeatStopwatch.Restart();
            if (!await _sendLock.WaitAsync(10)) return;
            try
            {
                await _ws.SendAsync(new ArraySegment<byte>(packet), WebSocketMessageType.Binary, true, _cts?.Token ?? CancellationToken.None);
                TotalPacketsTx++;
            }
            catch { }
            finally
            {
                _sendLock.Release();
            }
        }

        public async Task SendBinaryServoJointsAsync(double[] jointsDeg, byte gripper = 128)
        {
            if (_ws == null || _ws.State != WebSocketState.Open) return;

            byte[] packet = new byte[31];
            packet[0] = 0xAA;
            packet[1] = 0x01; // Opcode 0x01: SERVO_JOINTS streaming
            BitConverter.TryWriteBytes(packet.AsSpan(2, 2), (ushort)(_sendSeq++ & 0xFFFF));
            for (int i = 0; i < 6; i++)
            {
                float val = (jointsDeg != null && jointsDeg.Length > i) ? (float)jointsDeg[i] : 0f;
                BitConverter.TryWriteBytes(packet.AsSpan(4 + i * 4, 4), val);
            }
            packet[28] = gripper;
            packet[29] = 0x00;

            byte crc = 0;
            for (int i = 0; i < 30; i++) crc ^= packet[i];
            packet[30] = crc;

            // Frame drop if lock is held: prevents queue backlog during high frequency streaming
            if (!await _sendLock.WaitAsync(0)) return;
            try
            {
                await _ws.SendAsync(new ArraySegment<byte>(packet), WebSocketMessageType.Binary, true, _cts?.Token ?? CancellationToken.None);
                TotalPacketsTx++;
            }
            catch { }
            finally
            {
                _sendLock.Release();
            }
        }

        public async Task SendBinaryServoCartesianAsync(double[] poseMmDeg, byte gripper = 128)
        {
            if (_ws == null || _ws.State != WebSocketState.Open) return;

            byte[] packet = new byte[31];
            packet[0] = 0xAA;
            packet[1] = 0x02; // Opcode 0x02: SERVO_CARTESIAN streaming
            BitConverter.TryWriteBytes(packet.AsSpan(2, 2), (ushort)(_sendSeq++ & 0xFFFF));
            for (int i = 0; i < 6; i++)
            {
                float val = (poseMmDeg != null && poseMmDeg.Length > i) ? (float)poseMmDeg[i] : 0f;
                BitConverter.TryWriteBytes(packet.AsSpan(4 + i * 4, 4), val);
            }
            packet[28] = gripper;
            packet[29] = 0x00;

            byte crc = 0;
            for (int i = 0; i < 30; i++) crc ^= packet[i];
            packet[30] = crc;

            if (!await _sendLock.WaitAsync(0)) return;
            try
            {
                await _ws.SendAsync(new ArraySegment<byte>(packet), WebSocketMessageType.Binary, true, _cts?.Token ?? CancellationToken.None);
                TotalPacketsTx++;
            }
            catch { }
            finally
            {
                _sendLock.Release();
            }
        }

        #region High-Frequency Raw UDP Motion Streaming (Port 8766)

        public void SendUdpServoJoints(double[] jointsDeg, byte gripper = 128)
        {
            if (_udpClient == null) return;

            // 39-byte packed frame: [0xAA][Opcode 0x01][Seq: u16][Timestamp_ns: u64][6x float32: 24B][Gripper: u8][Flags: u8][CRC8: u8]
            byte[] packet = new byte[39];
            packet[0] = 0xAA;
            packet[1] = 0x01; // Opcode 0x01: SERVO_JOINTS streaming
            ushort seq = (ushort)(_sendSeq++ & 0xFFFF);
            BitConverter.TryWriteBytes(packet.AsSpan(2, 2), seq);
            ulong tsNs = GetCurrentTimestampNs();
            BitConverter.TryWriteBytes(packet.AsSpan(4, 8), tsNs);

            for (int i = 0; i < 6; i++)
            {
                float val = (jointsDeg != null && jointsDeg.Length > i) ? (float)jointsDeg[i] : 0f;
                BitConverter.TryWriteBytes(packet.AsSpan(12 + i * 4, 4), val);
            }
            packet[36] = gripper;
            packet[37] = 0x01; // Flags: Bit 0 = Continuous streaming active
            packet[38] = ComputeCrc8(packet.AsSpan(0, 38));

            try
            {
                _udpClient.Send(packet, packet.Length);
                UdpPacketsTx++;
                TotalPacketsTx++;
                if (jointsDeg != null && jointsDeg.Length >= 6)
                {
                    RawPacketLogged?.Invoke("UDP_TX", $"{{\"stream\":\"SERVO_JOINTS\",\"seq\":{seq},\"joints\":[{jointsDeg[0]:F2},{jointsDeg[1]:F2},{jointsDeg[2]:F2},{jointsDeg[3]:F2},{jointsDeg[4]:F2},{jointsDeg[5]:F2}],\"crc\":\"0x{packet[38]:X2}\"}}");
                }
            }
            catch { }
        }

        public void SendUdpServoCartesian(double[] poseMmDeg, byte gripper = 128)
        {
            if (_udpClient == null) return;

            byte[] packet = new byte[39];
            packet[0] = 0xAA;
            packet[1] = 0x02; // Opcode 0x02: SERVO_CARTESIAN streaming
            ushort seq = (ushort)(_sendSeq++ & 0xFFFF);
            BitConverter.TryWriteBytes(packet.AsSpan(2, 2), seq);
            ulong tsNs = GetCurrentTimestampNs();
            BitConverter.TryWriteBytes(packet.AsSpan(4, 8), tsNs);

            for (int i = 0; i < 6; i++)
            {
                float val = (poseMmDeg != null && poseMmDeg.Length > i) ? (float)poseMmDeg[i] : 0f;
                BitConverter.TryWriteBytes(packet.AsSpan(12 + i * 4, 4), val);
            }
            packet[36] = gripper;
            packet[37] = 0x03; // Flags: Bit 0 = Continuous streaming active, Bit 1 = Cartesian
            packet[38] = ComputeCrc8(packet.AsSpan(0, 38));

            try
            {
                _udpClient.Send(packet, packet.Length);
                UdpPacketsTx++;
                TotalPacketsTx++;
                if (poseMmDeg != null && poseMmDeg.Length >= 6)
                {
                    RawPacketLogged?.Invoke("UDP_TX", $"{{\"stream\":\"SERVO_CARTESIAN\",\"seq\":{seq},\"pose\":[{poseMmDeg[0]:F1},{poseMmDeg[1]:F1},{poseMmDeg[2]:F1},{poseMmDeg[3]:F1},{poseMmDeg[4]:F1},{poseMmDeg[5]:F1}],\"crc\":\"0x{packet[38]:X2}\"}}");
                }
            }
            catch { }
        }

        public void SendUdpHeartbeat()
        {
            if (_udpClient == null) return;

            byte[] packet = new byte[39];
            packet[0] = 0xAA;
            packet[1] = 0x05; // Opcode 0x05: Heartbeat
            ushort seq = (ushort)(_sendSeq++ & 0xFFFF);
            BitConverter.TryWriteBytes(packet.AsSpan(2, 2), seq);
            ulong tsNs = GetCurrentTimestampNs();
            BitConverter.TryWriteBytes(packet.AsSpan(4, 8), tsNs);
            packet[36] = 0;
            packet[37] = 0;
            packet[38] = ComputeCrc8(packet.AsSpan(0, 38));

            try
            {
                _udpClient.Send(packet, packet.Length);
                UdpPacketsTx++;
                TotalPacketsTx++;
            }
            catch { }
        }

        private async Task UdpReceiveLoopAsync()
        {
            int rxCount = 0;
            var rateSw = Stopwatch.StartNew();
            ushort lastSeq = 0;
            bool hasSeq = false;
            long lostPackets = 0;
            long totalExpected = 0;

            while (_cts != null && !_cts.IsCancellationRequested && _udpClient != null)
            {
                try
                {
                    UdpReceiveResult res = await _udpClient.ReceiveAsync(_cts.Token);
                    byte[] data = res.Buffer;
                    if (data == null || data.Length < 31 || data[0] != 0xAA) continue;

                    byte expectedCrc = data[data.Length - 1];
                    byte calcCrc = ComputeCrc8(data.AsSpan(0, data.Length - 1));
                    if (calcCrc != expectedCrc)
                    {
                        byte xor = 0;
                        for (int i = 0; i < data.Length - 1; i++) xor ^= data[i];
                        if (xor != expectedCrc) continue;
                    }

                    UdpPacketsRx++;
                    rxCount++;
                    if (rateSw.ElapsedMilliseconds >= 1000)
                    {
                        UdpRxRateHz = Math.Round(rxCount * 1000.0 / rateSw.ElapsedMilliseconds, 1);
                        rxCount = 0;
                        rateSw.Restart();
                    }

                    byte opcode = data[1];
                    ushort seq = BitConverter.ToUInt16(data, 2);

                    if (hasSeq)
                    {
                        int delta = (seq - lastSeq) & 0xFFFF;
                        if (delta > 1 && delta < 1000)
                        {
                            lostPackets += (delta - 1);
                            totalExpected += delta;
                        }
                        else
                        {
                            totalExpected += 1;
                        }
                        if (totalExpected > 10)
                        {
                            UdpPacketLossPercent = Math.Round((lostPackets * 100.0) / totalExpected, 1);
                        }
                    }
                    lastSeq = seq;
                    hasSeq = true;

                    if (opcode == 0x03 && data.Length >= 39)
                    {
                        ulong reflectedTsNs = BitConverter.ToUInt64(data, 4);
                        if (reflectedTsNs > 0)
                        {
                            ulong nowNs = GetCurrentTimestampNs();
                            if (nowNs > reflectedTsNs)
                            {
                                double rttMs = (nowNs - reflectedTsNs) / 1_000_000.0;
                                double jitterDelta = Math.Abs(rttMs - UdpRttMs);
                                UdpJitterMs = Math.Round(UdpJitterMs * 0.9 + jitterDelta * 0.1, 2);
                                UdpRttMs = Math.Round(rttMs, 2);
                                MeasuredRttMs = UdpRttMs;
                            }
                        }

                        float j1 = BitConverter.ToSingle(data, 12);
                        float j2 = BitConverter.ToSingle(data, 16);
                        float j3 = BitConverter.ToSingle(data, 20);
                        float j4 = BitConverter.ToSingle(data, 24);
                        float j5 = BitConverter.ToSingle(data, 28);
                        float j6 = BitConverter.ToSingle(data, 32);
                        byte gripper = data[36];
                        byte flags = data[37];

                        bool estop = (flags & 0x01) != 0;
                        bool homing = (flags & 0x02) != 0;
                        bool moving = (flags & 0x04) != 0;
                        bool homed = (flags & 0x08) != 0;

                        string state = estop ? "ESTOP" : homing ? "HOMING" : moving ? "STREAMING" : "IDLE";

                        var telem = new TelemetryData
                        {
                            State = state,
                            Homed = homed,
                            HomedMask = homed ? 0x3F : 0x00,
                            JointsDeg = new double[] { j1, j2, j3, j4, j5, j6 },
                            GripperMm = Math.Round(gripper / 2.55, 1),
                            KlipperConnected = true,
                            KlipperReady = true,
                            RttMs = UdpRttMs
                        };
                        TelemetryReceived?.Invoke(telem);
                        UdpMetricsUpdated?.Invoke(UdpRttMs, UdpJitterMs, UdpPacketLossPercent, UdpRxRateHz);
                    }
                    else if (opcode == 0x05 && data.Length >= 39)
                    {
                        ulong reflectedTsNs = BitConverter.ToUInt64(data, 4);
                        if (reflectedTsNs > 0)
                        {
                            ulong nowNs = GetCurrentTimestampNs();
                            if (nowNs > reflectedTsNs)
                            {
                                UdpRttMs = Math.Round((nowNs - reflectedTsNs) / 1_000_000.0, 2);
                                MeasuredRttMs = UdpRttMs;
                            }
                        }
                    }
                }
                catch (OperationCanceledException) { break; }
                catch (Exception) { }
            }
        }

        #endregion

        public Task SendSpeedLimitsAsync(double[] speedsDegS, double speedScale)
        {
            var speedsArray = new JsonArray();
            for (int i = 0; i < 6; i++)
            {
                speedsArray.Add(speedsDegS != null && speedsDegS.Length > i ? speedsDegS[i] : 40.0);
            }
            var payload = new JsonObject
            {
                ["cmd"] = "SET_SPEED_LIMITS",
                ["speed_scale"] = speedScale,
                ["speeds_deg_s"] = speedsArray
            };
            return SendJsonAsync(payload);
        }

        public async Task<bool> SendJsonAsync(object payload)
        {
            if (_ws == null || _ws.State != WebSocketState.Open) return false;

            string json = JsonSerializer.Serialize(payload);
            byte[] bytes = Encoding.UTF8.GetBytes(json);

            if (!await _sendLock.WaitAsync(200)) return false;
            try
            {
                if (_ws.State == WebSocketState.Open)
                {
                    await _ws.SendAsync(new ArraySegment<byte>(bytes), WebSocketMessageType.Text, true, CancellationToken.None);
                    TotalPacketsTx++;
                    RawPacketLogged?.Invoke("TX", json);
                    return true;
                }
            }
            catch (Exception ex)
            {
                LogMessage?.Invoke($"Send error: {ex.Message}");
            }
            finally
            {
                _sendLock.Release();
            }
            return false;
        }

        #region Outbound Control Actions

        public Task SendEstopAsync()
        {
            // Immediate ESTOP (dual action and cmd keys for instant M112 execution)
            return SendJsonAsync(new { action = "ESTOP", cmd = "ESTOP" });
        }

        public Task SendResetAsync()
        {
            // Dispatches FIRMWARE_RESTART to Klipper core and clears engine E-Stop
            return SendJsonAsync(new { action = "RESET", cmd = "RESET" });
        }

        public Task SendHomeAsync(string joint = "ALL")
        {
            return SendJsonAsync(new { action = "HOME", cmd = "HOME", joint = joint });
        }

        public Task SendSetCadHomeAsync()
        {
            return SendJsonAsync(new { action = "SET_CAD_HOME", cmd = "SET_CAD_HOME" });
        }

        public Task SendMoveJointsAsync(double[] joints, double duration = 0.5)
        {
            return SendJsonAsync(new
            {
                action = "MOVE_JOINTS",
                cmd = "MOVE_JOINTS",
                joints = joints,
                speed = 0.5,
                duration = duration,
                time = duration
            });
        }

        public Task SendServoJointsAsync(double[] joints, double dt = 0.02)
        {
            return SendJsonAsync(new
            {
                action = "SERVO_JOINTS",
                cmd = "SERVO_JOINTS",
                joints = joints,
                duration = dt,
                time = dt
            });
        }

        public Task SendMoveCartesianAsync(double xMm, double yMm, double zMm, double rollDeg, double pitchDeg, double yawDeg, double duration = 0.5)
        {
            return SendJsonAsync(new
            {
                action = "MOVE_CARTESIAN",
                cmd = "MOVE_CARTESIAN",
                target = new
                {
                    x = xMm / 1000.0,
                    y = yMm / 1000.0,
                    z = zMm / 1000.0,
                    roll = rollDeg,
                    pitch = pitchDeg,
                    yaw = yawDeg
                },
                pose = new[] { xMm, yMm, zMm, rollDeg, pitchDeg, yawDeg },
                duration = duration
            });
        }

        public Task SendJogJointAsync(int joint, double deltaDeg, double speed = 0.5)
        {
            return SendJsonAsync(new
            {
                action = "JOG_JOINT",
                cmd = "JOG_JOINT",
                joint = joint,
                delta_deg = deltaDeg,
                speed = speed
            });
        }

        public Task SendJogCartesianAsync(string axis, double deltaMm, double speed = 0.5)
        {
            return SendJsonAsync(new
            {
                action = "JOG_CARTESIAN",
                cmd = "JOG_CARTESIAN",
                axis = axis.ToUpper(),
                delta = deltaMm,
                speed = speed
            });
        }

        public Task SendSetGripperAsync(double positionAngle)
        {
            double clampedAngle = Math.Max(0.0, Math.Min(180.0, positionAngle));
            double gapMm = Math.Round(62.5 * (1.0 - (clampedAngle / 180.0)), 2);
            return SendJsonAsync(new
            {
                action = "SET_GRIPPER",
                cmd = "SET_GRIPPER",
                position = clampedAngle,
                angle = clampedAngle,
                mm = gapMm
            });
        }

        public Task SendGetTrimsAsync()
        {
            return SendJsonAsync(new { action = "GET_TRIMS", cmd = "GET_TRIMS" });
        }

        public Task SendClearLogsAsync()
        {
            return SendJsonAsync(new { action = "CLEAR_LOGS", cmd = "CLEAR_LOGS" });
        }

        public Task SendSetTrimsAsync(Dictionary<string, double> trims, bool save = true)
        {
            return SendJsonAsync(new
            {
                action = "SET_TRIMS",
                cmd = "SET_TRIMS",
                trims = trims,
                save = save
            });
        }

        public Task SendResetInvertMasksAsync()
        {
            return SendJsonAsync(new
            {
                action = "RESET_INVERT_MASKS",
                cmd = "RESET_INVERT_MASKS"
            });
        }

        public Task SendSetDirInvertAsync(int mask)
        {
            return SendJsonAsync(new
            {
                action = "SET_DIR_INVERT",
                cmd = "SET_DIR_INVERT",
                mask = mask
            });
        }

        public Task SendSetEndstopInvertAsync(int mask)
        {
            return SendJsonAsync(new
            {
                action = "SET_ENDSTOP_INVERT",
                cmd = "SET_ENDSTOP_INVERT",
                mask = mask
            });
        }

        #endregion

        public void Dispose()
        {
            Disconnect();
            _sendLock.Dispose();
        }
    }
}

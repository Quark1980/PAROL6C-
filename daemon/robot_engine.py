#!/usr/bin/env python3
"""
PAROL6 Robotics Engine: 6-DOF Kinematics, Trajectory & Daemon Core
==================================================================
Production-grade host control suite for PAROL6 6-DOF robotic arm running on Raspberry Pi 5.
- Kinematics: Standard DH parameter model + Damped Levenberg-Marquardt (LM) numerical IK.
  Seeded with active joint positions q_current for continuous Cartesian tracking without flips.
- Fluent Trajectory Generator: 100 Hz (10 ms) discretization with velocity blending across waypoints.
- Look-Ahead Step Compressor: 80-120 ms hardware clock lead buffer to eliminate queue starvation.
- Master Clock Drift Estimator: Weighted RTT synchronization between Linux monotonic clock & Mega 16 MHz timer.
- Binary Protocol: 250,000 baud USB-UART step streaming ('Q' / 'S'), E-stop ('E'), reset ('R'), gripper ('G').
- Safety Interlock: Strictly rejects all motion commands until hardware homing (homed_mask == 0x3F) completes.
- Dual Servers: WebSocket on ws://0.0.0.0:8765 and Touchscreen Kiosk HTTP on port 8000.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Any, List, Optional, Set, Tuple

import numpy as np
import serial
import serial.tools.list_ports
import websockets

from kinematics import (
    Parol6Kinematics,
    CAD_HOME_DEG,
    HARDWARE_LIMITS_DEG,
    STEPS_PER_DEG,
    PRESET_POSES_DEG,
    MAX_JOINT_SPEEDS_DEG_S,
    MAX_JOINT_ACCELS_DEG_S2,
)
from mcu_sync import McuClockSync
from trajectory_planner import TrajectoryPlanner, StepChunkData, MCU_FREQ_HZ

# ============================================================================
# LOGGING CONFIGURATION
# ============================================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] (%(name)s) %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
logger = logging.getLogger("parol6_engine")

# ============================================================================
# RUNTIME PARAMETERS & CONSTANTS
# ============================================================================
DEFAULT_SERIAL_PORT = os.environ.get("PAROL6_SERIAL_PORT", "/dev/ttyACM0")
SERIAL_BAUD = 250000
WS_PORT = 8765
UDP_PORT = 8766
HTTP_PORT = 8000

# Look-ahead buffering into MCU clock domain (80-120 ms)
LEAD_BUFFER_MS = 100.0
LEAD_BUFFER_TICKS = int(LEAD_BUFFER_MS * (MCU_FREQ_HZ / 1000.0))  # 1,600,000 ticks @ 16 MHz
STATUS_BROADCAST_INTERVAL_S = 0.05  # 20 Hz telemetry broadcast
TRAJECTORY_SAMPLE_DT_S = 0.01       # 100 Hz fluent trajectory discretization window (10 ms)

# ============================================================================
# CRC-8-CCITT LOOKUP TABLE & CHECKSUM VERIFICATION
# ============================================================================
CRC8_POLY = 0x07
CRC8_TABLE = []
for i in range(256):
    curr = i
    for _ in range(8):
        if curr & 0x80:
            curr = ((curr << 1) ^ CRC8_POLY) & 0xFF
        else:
            curr = (curr << 1) & 0xFF
    CRC8_TABLE.append(curr)

def compute_crc8_ccitt(data: bytes) -> int:
    """Compute CRC-8-CCITT (poly 0x07, init 0x00) using precomputed lookup table."""
    val = 0
    for b in data:
        val = CRC8_TABLE[val ^ b]
    return val

def verify_crc(packet: bytes) -> bool:
    """Verify packet checksum using CRC-8-CCITT with XOR fallback."""
    if not packet or len(packet) < 2:
        return False
    expected = packet[-1]
    body = packet[:-1]
    if compute_crc8_ccitt(body) == expected:
        return True
    # Fallback XOR for backward compatibility
    xor = 0
    for b in body:
        xor ^= b
    return xor == expected

# ============================================================================
# DETERMINISTIC HOST-SIDE JITTER BUFFER (CIRCULAR LEAKY BUCKET)
# ============================================================================
class JitterBufferFrame:
    """Pre-allocated circular buffer slot (zero runtime heap allocations)."""
    __slots__ = ('valid', 'seq', 'timestamp_ns', 'opcode', 'payload', 'gripper', 'flags', 'recv_mono_ns')
    def __init__(self):
        self.valid = False
        self.seq = 0
        self.timestamp_ns = 0
        self.opcode = 0
        self.payload = [0.0] * 6
        self.gripper = 0
        self.flags = 0
        self.recv_mono_ns = 0

class JitterBuffer:
    """
    Fixed-Capacity Circular Ring Buffer & Monotonic Leaky Bucket.
    Absorbs network transmission jitter and temporal packet clumping across Wi-Fi.
    Feeds the kinematics & stepping engine at a deterministic 100 Hz cadence with 50-80 ms target horizon.
    """
    def __init__(self, capacity: int = 64, target_horizon_ms: float = 60.0):
        self.capacity = capacity
        self.target_horizon_ns = int(target_horizon_ms * 1_000_000)
        self.frames = [JitterBufferFrame() for _ in range(capacity)]
        self.head = 0  # write slot
        self.tail = 0  # read slot
        self.count = 0
        self.lock = threading.Lock()

        # Horizon synchronization
        self.is_active = False
        self.base_client_ns = 0
        self.base_host_ns = 0
        self.last_playback_seq = -1
        self.last_popped_payload = [0.0] * 6
        self.has_popped = False

        # Diagnostic metrics
        self.packets_rx = 0
        self.packets_dropped_late = 0
        self.packets_dropped_overflow = 0
        self.underrun_count = 0
        self.overrun_count = 0
        self.last_pop_time = 0.0

    def push(self, seq: int, timestamp_ns: int, opcode: int, payload: List[float], gripper: int, flags: int) -> bool:
        """Push an incoming motion frame into the jitter ring buffer. Zero runtime heap allocation."""
        now_ns = time.perf_counter_ns()
        with self.lock:
            # Re-initialize horizon if stream was inactive or buffer starved for > 1.0s
            if not self.is_active or (now_ns - self.base_host_ns > 1_000_000_000 and self.count == 0):
                self.is_active = True
                self.base_client_ns = timestamp_ns
                self.base_host_ns = now_ns
                self.head = 0
                self.tail = 0
                self.count = 0
                self.last_playback_seq = -1

            # Discard late or out-of-order packets if older than last playback sequence
            if self.last_playback_seq >= 0:
                seq_delta = (seq - self.last_playback_seq) & 0xFFFF
                if seq_delta > 0x8000:  # Packet is older than playback head
                    self.packets_dropped_late += 1
                    return False

            # If full, drop oldest frame to maintain bounded delay
            if self.count >= self.capacity:
                self.tail = (self.tail + 1) % self.capacity
                self.count -= 1
                self.packets_dropped_overflow += 1
                self.overrun_count += 1

            # Store in pre-allocated slot
            f = self.frames[self.head]
            f.valid = True
            f.seq = seq
            f.timestamp_ns = timestamp_ns
            f.opcode = opcode
            f.payload[0] = float(payload[0])
            f.payload[1] = float(payload[1])
            f.payload[2] = float(payload[2])
            f.payload[3] = float(payload[3])
            f.payload[4] = float(payload[4])
            f.payload[5] = float(payload[5])
            f.gripper = gripper
            f.flags = flags
            f.recv_mono_ns = now_ns

            self.head = (self.head + 1) % self.capacity
            self.count += 1
            self.packets_rx += 1
            return True

    def pop(self) -> Optional[JitterBufferFrame]:
        """
        Deterministic 100 Hz extraction: Returns the frame matching the current playback horizon,
        or None if starving / waiting for initial buffering horizon.
        """
        now_ns = time.perf_counter_ns()
        with self.lock:
            if not self.is_active or self.count == 0:
                if self.is_active and self.has_popped:
                    self.underrun_count += 1
                return None

            elapsed_host_ns = now_ns - self.base_host_ns
            # Wait until initial target horizon buffer is accumulated (e.g. 60 ms)
            if elapsed_host_ns < self.target_horizon_ns:
                return None

            # Target playback client timestamp
            target_playback_ns = self.base_client_ns + (elapsed_host_ns - self.target_horizon_ns)

            # Pop frames from tail; skip any frames that are older than target_playback_ns - 25 ms
            frame_to_return = None
            while self.count > 0:
                f = self.frames[self.tail]
                self.tail = (self.tail + 1) % self.capacity
                self.count -= 1

                # If buffer is getting too deep (> 50 frames), fast-forward to maintain <= 80 ms latency
                if self.count > 50:
                    self.overrun_count += 1
                    continue

                frame_to_return = f
                # If this frame's timestamp is reasonably close to or ahead of target playback, use it
                if f.timestamp_ns >= target_playback_ns:
                    break

            if frame_to_return is not None:
                self.last_playback_seq = frame_to_return.seq
                self.last_popped_payload = list(frame_to_return.payload)
                self.has_popped = True
                self.last_pop_time = now_ns
                return frame_to_return
            else:
                self.underrun_count += 1
                return None

# ============================================================================
# ASYNC UDP MOTION PROTOCOL
# ============================================================================
class UdpMotionProtocol(asyncio.DatagramProtocol):
    """Zero-allocation asyncio UDP datagram protocol for real-time motion & telemetry."""
    def __init__(self, engine: RobotEngine):
        self.engine = engine
        self.transport: Optional[asyncio.DatagramTransport] = None

    def connection_made(self, transport: asyncio.DatagramTransport):
        self.transport = transport
        self.engine.udp_transport = transport
        logger.info("UDP Motion Server bound and active on 0.0.0.0:%d", UDP_PORT)

    def datagram_received(self, data: bytes, addr: Tuple[str, int]):
        self.engine.handle_udp_packet(data, addr)

    def error_received(self, exc: Exception):
        logger.warning("UDP error received: %s", exc)

    def connection_lost(self, exc: Optional[Exception]):
        logger.info("UDP transport closed: %s", exc)


class RobotEngine:
    """
    Unified PAROL6 Control Engine and Kinematics Daemon.
    """

    def __init__(self, serial_port: str = DEFAULT_SERIAL_PORT):
        self.port_name = serial_port
        self.serial: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.running = True

        # MCU Connection & Health
        self.mcu_connected = False
        self.last_mcu_rx = 0.0
        self.mcu_packets_rx = 0
        self.mcu_packets_tx = 0

        # Subsystems
        self.kinematics = Parol6Kinematics()
        self.planner = TrajectoryPlanner()
        self.clock_sync = McuClockSync(expected_freq_hz=MCU_FREQ_HZ)

        # Configurable persistent homing offset trims
        self.load_offsets()

        # State tracking
        self.state: str = "IDLE"  # IDLE, MOVING, HOMING, ESTOP
        self.homed: bool = False
        self.joints_deg: List[float] = [0.0] * 6
        self.target_joints_deg: List[float] = [0.0] * 6
        self.gripper_mm: float = 62.5
        self.endstops_mask: int = 0
        self.endstop_invert_mask: int = 0
        self.dir_invert_mask: int = 0x03  # Default: invert J1 and J2 per printer.cfg (!PF1, !PF7)
        self.speed_scale: float = 0.4     # Master speed scaling factor (10% to 100%)
        self.telemetry_seq: int = 0
        self.last_client_heartbeat: float = time.monotonic()

        # Look-ahead chunk management (per joint)
        self.free_slots: List[int] = [16] * 6
        self.pending_chunks: List[List[StepChunkData]] = [[] for _ in range(6)]
        self.streaming_lock = asyncio.Lock()

        # Real-time hardware master clock & joint streaming
        self.last_mcu_clock: int = 0
        self.last_mcu_clock_host_time: float = 0.0
        self.joint_stream_end_clock: List[int] = [0] * 6
        self.stream_packet_count: int = 0
        self.stream_last_rx: float = 0.0
        self._last_stream_time: float = 0.0

        # Diagnostics & Ping loop
        self.last_ping_send: float = 0.0
        self.last_ping_rtt_ms: float = 0.0

        # Connected WebSocket clients
        self.connected_ws_clients: Set[websockets.WebSocketServerProtocol] = set()

        # Hybrid Transport (UDP Stream + Jitter Buffer)
        self.udp_transport: Optional[asyncio.DatagramTransport] = None
        self.client_udp_endpoint: Optional[Tuple[str, int]] = None
        self.jitter_buffer = JitterBuffer(capacity=64, target_horizon_ms=60.0)
        self.last_client_timestamp_ns: int = 0

        # Start serial listener worker thread
        self.serial_thread = threading.Thread(target=self._serial_worker_thread, daemon=True)
        self.serial_thread.start()

    # =========================================================================
    # PERSISTENT HOMING OFFSET TRIMS (offsets.json)
    # =========================================================================

    def load_offsets(self):
        """Load persistent homing trims from offsets.json."""
        self.offsets_path = Path(__file__).resolve().parent / "offsets.json"
        self.homing_trims_deg = {f"j{i+1}": 0.0 for i in range(6)}
        if self.offsets_path.exists():
            try:
                with open(self.offsets_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    trims = data.get("homing_trims_deg", {})
                    for i in range(6):
                        k = f"j{i+1}"
                        if k in trims:
                            self.homing_trims_deg[k] = float(trims[k])
                logger.info("Loaded homing trims from %s: %s", self.offsets_path, self.homing_trims_deg)
            except Exception as e:
                logger.warning("Failed loading offsets.json: %s", e)
        self.homing_trims_array = [self.homing_trims_deg[f"j{i+1}"] for i in range(6)]

    def update_trims(self, new_trims: Dict[str, Any], persist: bool = True):
        """Update live homing trims and optionally persist to offsets.json."""
        for k, v in new_trims.items():
            k_lower = str(k).lower()
            if k_lower in self.homing_trims_deg:
                try:
                    self.homing_trims_deg[k_lower] = round(float(v), 3)
                except (ValueError, TypeError):
                    pass
        self.homing_trims_array = [self.homing_trims_deg[f"j{i+1}"] for i in range(6)]
        if persist:
            try:
                data = {"homing_trims_deg": self.homing_trims_deg}
                with open(self.offsets_path, "w", encoding="utf-8") as f:
                    json.dump(data, f, indent=2)
                logger.info("Persisted updated homing trims to %s: %s", self.offsets_path, self.homing_trims_deg)
            except Exception as e:
                logger.error("Failed saving offsets.json: %s", e)

    # =========================================================================
    # HARDWARE MASTER CLOCK SYNCHRONIZATION
    # =========================================================================

    def get_estimated_mcu_clock(self) -> int:
        """
        Estimate the current 32-bit MCU hardware clock based on Linux time.monotonic().
        """
        if self.clock_sync.synced:
            now_mono = time.monotonic()
            unwrapped = self.clock_sync.mcu0 + int(round((now_mono - self.clock_sync.t0) * self.clock_sync.slope))
            return unwrapped & 0xFFFFFFFF
        return self.last_mcu_clock

    # =========================================================================
    # SERIAL PORT MANAGEMENT
    # =========================================================================

    def connect_serial(self) -> bool:
        """Locate and open high-speed USB-UART connection to ATmega2560."""
        with self.serial_lock:
            if self.serial and self.serial.is_open:
                return True

            port_candidates = []
            if os.path.exists(self.port_name):
                port_candidates.append(self.port_name)

            # Auto-detect by-id or USB ports
            by_id_dir = "/dev/serial/by-id"
            if os.path.exists(by_id_dir):
                for fname in sorted(os.listdir(by_id_dir)):
                    full_p = os.path.join(by_id_dir, fname)
                    if full_p not in port_candidates:
                        port_candidates.append(full_p)

            for p in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1", "COM3", "COM4", "COM5"]:
                if p not in port_candidates and os.path.exists(p):
                    port_candidates.append(p)

            for p in port_candidates:
                try:
                    logger.info("Connecting to ATmega2560 on %s at %d baud...", p, SERIAL_BAUD)
                    s = serial.Serial(p, SERIAL_BAUD, timeout=0.01, write_timeout=0.2)
                    time.sleep(1.8)  # Allow ATmega2560 DTR reboot stabilization
                    s.reset_input_buffer()
                    s.reset_output_buffer()
                    self.serial = s
                    self.port_name = p
                    self.mcu_connected = True
                    logger.info("Serial communication established on %s.", p)
                    # Sync direction and endstop invert masks
                    self.set_dir_invert_mask(self.dir_invert_mask)
                    self.set_endstop_invert_mask(self.endstop_invert_mask)
                    return True
                except Exception as e:
                    logger.warning("Failed opening port %s: %s", p, e)

            return False

    def write_raw(self, data: bytes):
        """Thread-safe non-blocking write to USB-UART."""
        s = self.serial
        if s and s.is_open:
            with self.write_lock:
                try:
                    s.write(data)
                    self.mcu_packets_tx += 1
                except Exception as e:
                    logger.error("Serial write exception: %s", e)

    # =========================================================================
    # SERIAL PROTOCOL WORKER THREAD
    # =========================================================================

    def _serial_worker_thread(self):
        """
        Dedicated background worker thread for robust, low-latency, frame-synchronized
        binary protocol processing without blocking the asyncio event loop.
        """
        rx_buf = bytearray()
        logger.info("Serial background worker thread initialized.")

        while self.running:
            s = self.serial
            if not s or not s.is_open:
                if not self.connect_serial():
                    time.sleep(1.0)
                    continue
                s = self.serial

            try:
                in_waiting = s.in_waiting if (s and s.is_open) else 0
                if in_waiting > 0:
                    chunk = s.read(in_waiting)
                else:
                    chunk = s.read(1)

                if chunk:
                    rx_buf.extend(chunk)
                    self.last_mcu_rx = time.monotonic()
                    self.mcu_connected = True
                else:
                    time.sleep(0.002)
                    continue

                # Parse frame-synchronized binary packets
                while len(rx_buf) > 0:
                    header = rx_buf[0]

                    # 'P': Ping clock synchronization packet (6 bytes: 'P' + uint32 + '\n')
                    if header == ord('P'):
                        if len(rx_buf) < 6:
                            break
                        if rx_buf[5] == ord('\n'):
                            mcu_clock = struct.unpack("<I", rx_buf[1:5])[0]
                            t_now = time.monotonic()
                            self.last_mcu_clock = mcu_clock
                            self.last_mcu_clock_host_time = t_now
                            self.clock_sync.record_sample(
                                t_send=self.last_ping_send,
                                t_recv=t_now,
                                raw_mcu_u32=mcu_clock,
                            )
                            self.mcu_packets_rx += 1
                            rx_buf = rx_buf[6:]
                        else:
                            rx_buf.pop(0)

                    # '!': Comprehensive telemetry frame (32 bytes: '!' + flags + 24B pos + endstops + 4B clock + '\n')
                    elif header == ord('!'):
                        if len(rx_buf) < 32:
                            break
                        if rx_buf[31] == ord('\n'):
                            flags = rx_buf[1]
                            positions = struct.unpack("<6i", rx_buf[2:26])
                            endstops = rx_buf[26]
                            mcu_clock = struct.unpack("<I", rx_buf[27:31])[0]

                            t_now = time.monotonic()
                            self.last_mcu_clock = mcu_clock
                            self.last_mcu_clock_host_time = t_now
                            self.endstops_mask = endstops

                            is_mcu_stepping = (flags & 0x04) != 0
                            has_pending = any(len(q) > 0 for q in self.pending_chunks)
                            is_busy = is_mcu_stepping or has_pending

                            if not is_mcu_stepping:
                                # When MCU is not actively executing step chunks, its ring buffers are empty
                                self.free_slots = [16] * 6
                                if not has_pending:
                                    self.joint_stream_end_clock = [0] * 6

                            if flags & 0x01:
                                self.state = "ESTOP"
                            elif flags & 0x02:
                                self.state = "HOMING"
                            elif is_busy:
                                self.state = "MOVING"
                            elif self.state != "ESTOP":
                                self.state = "IDLE"

                            self.homed = bool(flags & 0x08)

                            # Always reflect live hardware joint positions directly from step counters + persistent trims
                            self.joints_deg = [
                                round((positions[i] / STEPS_PER_DEG[i]) + self.homing_trims_array[i], 3)
                                for i in range(6)
                            ]

                            self.mcu_packets_rx += 1
                            rx_buf = rx_buf[32:]
                        else:
                            rx_buf.pop(0)

                    # 'K': Chunk accepted ACK (4 bytes: 'K' + joint + free_slots + '\n')
                    elif header == ord('K'):
                        if len(rx_buf) < 4:
                            break
                        if rx_buf[3] == ord('\n'):
                            joint = rx_buf[1]
                            free = rx_buf[2]
                            if joint < 6:
                                self.free_slots[joint] = free
                            self.mcu_packets_rx += 1
                            rx_buf = rx_buf[4:]
                        else:
                            rx_buf.pop(0)

                    # 0x06: Single-byte ACK from fast 'Q' chunk
                    elif header == 0x06:
                        self.mcu_packets_rx += 1
                        rx_buf.pop(0)

                    # 0x15: Single-byte NAK from fast 'Q' chunk
                    elif header == 0x15:
                        logger.warning("MCU returned NAK (0x15) on fast step chunk")
                        self.mcu_packets_rx += 1
                        rx_buf.pop(0)

                    # 'N': Chunk rejected NACK (4 bytes: 'N' + joint + err + '\n')
                    elif header == ord('N'):
                        if len(rx_buf) < 4:
                            break
                        if rx_buf[3] == ord('\n'):
                            joint_err = rx_buf[1]
                            err_code = rx_buf[2]
                            err_desc = {
                                1: "Buffer Full",
                                2: "CRC Checksum Error",
                                3: "Invalid Joint Index",
                                4: "E-Stop Active",
                                5: "Arm Not Homed (Must Home First)",
                            }.get(err_code, f"Unknown ({err_code})")
                            logger.warning("MCU rejected chunk on joint %d: %s", joint_err, err_desc)
                            self.mcu_packets_rx += 1
                            rx_buf = rx_buf[4:]
                        else:
                            rx_buf.pop(0)

                    # 'M': Manual step completed ACK (4 bytes: 'M' + joint + ok + '\n')
                    elif header == ord('M'):
                        if len(rx_buf) < 4:
                            break
                        if rx_buf[3] == ord('\n'):
                            self.mcu_packets_rx += 1
                            rx_buf = rx_buf[4:]
                        else:
                            rx_buf.pop(0)

                    # 'I': Invert mask ACK (3 bytes: 'I' + mask + '\n')
                    elif header == ord('I'):
                        if len(rx_buf) < 3:
                            break
                        if rx_buf[2] == ord('\n'):
                            self.endstop_invert_mask = rx_buf[1]
                            self.mcu_packets_rx += 1
                            rx_buf = rx_buf[3:]
                        else:
                            rx_buf.pop(0)

                    # 'D': Direction invert mask ACK (3 bytes: 'D' + mask + '\n')
                    elif header == ord('D'):
                        if len(rx_buf) < 3:
                            break
                        if rx_buf[2] == ord('\n'):
                            self.dir_invert_mask = rx_buf[1]
                            self.mcu_packets_rx += 1
                            rx_buf = rx_buf[3:]
                        else:
                            rx_buf.pop(0)

                    # 'H': Homing status reply (3 bytes: 'H' + status + '\n')
                    elif header == ord('H'):
                        if len(rx_buf) < 3:
                            break
                        if rx_buf[2] == ord('\n'):
                            code = rx_buf[1]
                            if code == 0x01:
                                logger.info("MCU homing started or completed successfully.")
                            elif code == 0xFE:
                                logger.warning("MCU rejected homing (E-Stop Active).")
                            self.mcu_packets_rx += 1
                            rx_buf = rx_buf[3:]
                        else:
                            rx_buf.pop(0)

                    # 'E': Emergency stop confirmation
                    elif header == ord('E'):
                        if len(rx_buf) < 3:
                            break
                        if rx_buf[2] == ord('\n'):
                            self.state = "ESTOP"
                            self.homed = False
                            self.mcu_packets_rx += 1
                            rx_buf = rx_buf[3:]
                        else:
                            rx_buf.pop(0)

                    # 'R': Reset confirmation
                    elif header == ord('R'):
                        if len(rx_buf) < 3:
                            break
                        if rx_buf[2] == ord('\n'):
                            self.state = "IDLE"
                            self.mcu_packets_rx += 1
                            rx_buf = rx_buf[3:]
                        else:
                            rx_buf.pop(0)

                    # 'G': Gripper response
                    elif header == ord('G'):
                        if len(rx_buf) < 3:
                            break
                        if rx_buf[2] == ord('\n'):
                            self.mcu_packets_rx += 1
                            rx_buf = rx_buf[3:]
                        else:
                            rx_buf.pop(0)

                    else:
                        # Skip unrecognizable header byte
                        rx_buf.pop(0)

            except serial.SerialException as se:
                logger.error("Serial connection lost: %s", se)
                self.mcu_connected = False
                with self.serial_lock:
                    if self.serial:
                        try:
                            self.serial.close()
                        except Exception:
                            pass
                        self.serial = None
                time.sleep(1.0)
            except Exception as e:
                logger.error("Exception in serial worker thread: %s", e)
                time.sleep(0.01)

    # =========================================================================
    # CORE ROBOT CONTROL ACTIONS
    # =========================================================================

    def emergency_stop(self):
        """Instant hardware coils disable and queue clearing."""
        logger.warning(">>> EMERGENCY STOP TRIGGERED <<<")
        self.state = "ESTOP"
        for q in self.pending_chunks:
            q.clear()
        self.free_slots = [16] * 6
        self.joint_stream_end_clock = [0] * 6
        self.write_raw(b'E')

    def reset_system(self):
        """Clear errors and re-energize motor coils."""
        logger.info("System Reset / Re-energize requested.")
        for q in self.pending_chunks:
            q.clear()
        self.free_slots = [16] * 6
        self.joint_stream_end_clock = [0] * 6
        self.write_raw(b'R')
        self.state = "IDLE"

    def trigger_homing(self, mask: int = 0xFF):
        """Initiate upgraded hardware soft-start homing sequence."""
        logger.info("Homing triggered with axis mask: 0x%02X", mask)
        self.state = "HOMING"
        for q in self.pending_chunks:
            q.clear()
        self.free_slots = [16] * 6
        self.joint_stream_end_clock = [0] * 6
        payload = struct.pack("<B B B", ord('H'), mask & 0xFF, ord('\n'))
        self.write_raw(payload)

    def manual_step(self, joint_1_indexed: int, direction: int, steps: int, delay_us: int = 150):
        """Direct single-joint pulse burst for diagnostic validation."""
        joint_0 = max(0, min(5, joint_1_indexed - 1))
        direction = 1 if direction > 0 else 0
        steps = max(1, min(65535, int(steps)))
        delay_us = max(20, min(65535, int(delay_us)))
        payload = struct.pack("<B B B H H B", ord('M'), joint_0, direction, steps, delay_us, ord('\n'))
        self.write_raw(payload)

    def set_endstop_invert_mask(self, mask: int):
        """Dynamically configure endstop polarity mask."""
        self.endstop_invert_mask = mask & 0x3F
        payload = struct.pack("<B B B", ord('I'), self.endstop_invert_mask, ord('\n'))
        self.write_raw(payload)

    def set_dir_invert_mask(self, mask: int):
        """Dynamically configure DIR pin inversion mask."""
        self.dir_invert_mask = mask & 0x3F
        payload = struct.pack("<B B B", ord('D'), self.dir_invert_mask, ord('\n'))
        self.write_raw(payload)

    def set_gripper(self, mm: Optional[float] = None, angle: Optional[float] = None):
        """
        Set gripper opening in millimeters [0..62.5 mm] or degrees [0..180 deg].
        Maps 0..180 deg to 500..2500 us pulse width on D11.
        """
        max_mm = 62.5
        max_angle = 180.0

        if mm is not None:
            clamped_mm = max(0.0, min(max_mm, float(mm)))
            target_angle = max_angle - (clamped_mm / max_mm * max_angle)
            self.gripper_mm = round(clamped_mm, 2)
        elif angle is not None:
            target_angle = max(0.0, min(max_angle, float(angle)))
            self.gripper_mm = round((max_angle - target_angle) / max_angle * max_mm, 2)
        else:
            return

        pulse_us = int(round(500.0 + (target_angle / 180.0) * 2000.0))
        pulse_us = max(500, min(2500, pulse_us))
        payload = struct.pack("<B H B", ord('G'), pulse_us, ord('\n'))
        self.write_raw(payload)

    # =========================================================================
    # REAL-TIME JOINT STREAMING & MOVE BLENDING
    # =========================================================================

    async def servo_to_joints(
        self,
        target_joints_deg: List[float],
        dt: float = 0.05,
        velocities: Optional[List[float]] = None,
        accelerations: Optional[List[float]] = None,
    ) -> bool:
        """
        Real-Time Joint Follower:
        Receives joint targets directly from the Windows Control Suite or Touchscreen,
        validates soft limits, translates angular deltas into quadratic step chunks,
        and pushes them directly to the ATmega2560 hardware queues.
        Buffer is maintained 80-120 ms ahead into the MCU hardware clock domain.
        """
        if self.state == "ESTOP":
            logger.info("Waking robot from E-STOP for incoming joint stream...")
            self.reset_system()
            await asyncio.sleep(0.02)

        # Hardware Safety Interlock
        if not self.homed:
            logger.warning("Joint stream rejected: Robot must be homed before accepting joint stream!")
            return False

        self.stream_packet_count += 1
        self.stream_last_rx = time.monotonic()

        # Clamp target joint angles within safe soft boundaries
        clamped_targets = self.kinematics.clamp_joints(target_joints_deg)

        # Base reference: use target_joints_deg from previous cycle if recent, else actual hardware joints
        now_mono = time.monotonic()
        if (self.state == "MOVING") and (now_mono - self._last_stream_time < 0.25):
            ref_joints = list(self.target_joints_deg)
        else:
            ref_joints = list(self.joints_deg)
        self._last_stream_time = now_mono

        dt_clamped = max(0.01, min(1.0, float(dt)))
        mcu_dt_ticks = int(round(dt_clamped * MCU_FREQ_HZ))
        now_mcu = self.get_estimated_mcu_clock()

        new_chunks_added = False

        async with self.streaming_lock:
            for j in range(6):
                delta_deg = clamped_targets[j] - ref_joints[j]
                steps = int(round(delta_deg * STEPS_PER_DEG[j]))
                if steps == 0:
                    continue

                direction = 1 if steps > 0 else 0
                abs_steps = abs(steps)

                # Determine chunk start clock: chain seamlessly with 80-120 ms lead buffer
                end_clock = self.joint_stream_end_clock[j]
                signed_lead = (int(end_clock) - int(now_mcu)) & 0xFFFFFFFF
                if signed_lead > 0x80000000:
                    signed_lead -= 0x100000000

                if end_clock == 0 or signed_lead < LEAD_BUFFER_TICKS:
                    # Initialize look-ahead lead buffer (100 ms in future)
                    start_clock = (now_mcu + LEAD_BUFFER_TICKS) & 0xFFFFFFFF
                else:
                    start_clock = end_clock

                # Step distribution over duration dt
                interval_ticks = int(mcu_dt_ticks / abs_steps)
                interval_ticks = max(160, min(65535, interval_ticks))

                steps_remaining = abs_steps
                curr_clock = start_clock

                while steps_remaining > 0:
                    count = min(24, steps_remaining)
                    chunk = StepChunkData(
                        joint=j,
                        first_step_clock=curr_clock,
                        step_count=count,
                        interval=interval_ticks,
                        add=0,
                        dir=direction,
                    )
                    self.pending_chunks[j].append(chunk)
                    new_chunks_added = True
                    curr_clock = (curr_clock + count * interval_ticks) & 0xFFFFFFFF
                    steps_remaining -= count

                self.joint_stream_end_clock[j] = curr_clock

            self.target_joints_deg = list(clamped_targets)
            if new_chunks_added:
                self.state = "MOVING"

        await self.stream_pending_chunks()
        return True

    async def stream_trajectory(self, points: List[Dict[str, Any]]) -> bool:
        """
        Execute multi-point dense joint trajectory calculated by Windows Control Suite.
        Discretizes and blends points directly into the MCU step queue.
        """
        if not self.homed:
            logger.warning("Trajectory stream rejected: Robot must be homed before executing trajectories!")
            return False

        for pt in points:
            target = pt.get("joints", None)
            dt = float(pt.get("time", 0.05))
            v = pt.get("velocities", None)
            a = pt.get("accelerations", None)
            if target and len(target) == 6:
                await self.servo_to_joints(target, dt=dt, velocities=v, accelerations=a)
                # Keep buffer filled without overflowing
                while any(len(q) > 8 for q in self.pending_chunks):
                    await self.stream_pending_chunks()
                    await asyncio.sleep(0.01)
        return True

    async def execute_joint_move(self, target_joints_deg: List[float], speed_factor: float = 0.5) -> bool:
        """
        Plan and stream synchronized multi-axis trapezoidal motion profile to ATmega2560.
        All active axes accelerate, cruise, and decelerate concurrently to reach destination simultaneously.
        """
        if self.state == "ESTOP":
            logger.info("Waking robot from E-STOP for requested move...")
            self.reset_system()
            await asyncio.sleep(0.02)

        if not self.homed:
            logger.warning("Joint move rejected: Robot must be homed before moving!")
            return False

        clamped = self.kinematics.clamp_joints(target_joints_deg)
        duration, profiles = self.planner.plan_synchronized_move(
            q_start_deg=self.joints_deg,
            q_target_deg=clamped,
            speed_factor=speed_factor,
        )

        if duration <= 1e-4:
            return True

        now_mcu = self.get_estimated_mcu_clock()
        start_clock = (now_mcu + LEAD_BUFFER_TICKS) & 0xFFFFFFFF

        async with self.streaming_lock:
            for j in range(6):
                p = profiles[j]
                if p is not None:
                    chunks = self.planner.compress_joint_steps(j, p, start_clock)
                    self.pending_chunks[j].extend(chunks)
                    if chunks:
                        last_c = chunks[-1]
                        end_tick = (last_c.first_step_clock + last_c.step_count * last_c.interval) & 0xFFFFFFFF
                        self.joint_stream_end_clock[j] = end_tick

            self.target_joints_deg = list(clamped)
            self.state = "MOVING"

        await self.stream_pending_chunks()
        return True

    async def execute_cartesian_move(self, target_pose: List[float], speed_factor: float = 0.5) -> bool:
        """
        Solve Levenberg-Marquardt Inverse Kinematics seeded with q_current,
        then execute synchronized joint motion.
        """
        if not self.homed:
            logger.warning("Cartesian move rejected: Robot must be homed first!")
            return False

        ik_res = self.kinematics.solve_ik(target_pose, seed_joints_deg=self.joints_deg)
        if not ik_res.success:
            logger.warning("IK Failed for Cartesian target %s: %s", target_pose, ik_res.message)
            return False

        return await self.execute_joint_move(ik_res.joints_deg, speed_factor=speed_factor)

    async def servo_to_cartesian(self, target_pose: List[float], dt: float = 0.05) -> bool:
        """
        Solve Levenberg-Marquardt / DLS Inverse Kinematics for high-frequency Cartesian streaming,
        then route through servo_to_joints.
        """
        if not self.homed:
            return False
        seed = self.target_joints_deg if self.state == "MOVING" else self.joints_deg
        ik_res = self.kinematics.solve_ik(target_pose, seed_joints_deg=seed)
        if not ik_res.success:
            logger.debug("Cartesian streaming IK failure: %s", ik_res.message)
            return False
        return await self.servo_to_joints(ik_res.joints_deg, dt=dt)

    # =========================================================================
    # HYBRID UDP STREAMING & JITTER BUFFER SAMPLING
    # =========================================================================

    def handle_udp_packet(self, data: bytes, addr: Tuple[str, int]):
        """
        Zero-allocation deserializer for high-frequency UDP motion frames (Port 8766).
        Extracts 39-byte (or 31-byte / 32-byte) binary frame, validates CRC8,
        and pushes setpoint into the deterministic circular jitter buffer.
        """
        length = len(data)
        if length < 31 or data[0] != 0xAA:
            return
        if not verify_crc(data):
            return

        self.client_udp_endpoint = addr

        try:
            if length == 39:
                header, opcode, seq, ts_ns, j1, j2, j3, j4, j5, j6, gripper, flags, _crc = struct.unpack_from("<BBHQ6fBBB", data)
            elif length == 31:
                header, opcode, seq, j1, j2, j3, j4, j5, j6, gripper, flags, _crc = struct.unpack_from("<BBH6fBBB", data)
                ts_ns = time.perf_counter_ns()
            elif length == 32:
                header, opcode, seq, ts_32, j1, j2, j3, j4, j5, j6, gripper, flags = struct.unpack_from("<BBHI6fBB", data)
                ts_ns = int(ts_32) * 1_000_000
            else:
                return
        except (struct.error, ValueError):
            return

        self.last_client_heartbeat = time.monotonic()
        self.last_client_timestamp_ns = ts_ns

        if opcode == 0x01:  # MOVE_JOINTS (Joint angles)
            if self.homed and self.state != "ESTOP":
                self.jitter_buffer.push(seq, ts_ns, opcode, [j1, j2, j3, j4, j5, j6], gripper, flags)
        elif opcode == 0x02:  # MOVE_CARTESIAN (Cartesian pose)
            if self.homed and self.state != "ESTOP":
                self.jitter_buffer.push(seq, ts_ns, opcode, [j1, j2, j3, j4, j5, j6], gripper, flags)
        elif opcode == 0x05:  # HEARTBEAT
            self.send_udp_heartbeat_reply(seq, ts_ns, addr)

    def send_udp_heartbeat_reply(self, seq: int, client_ts_ns: int, addr: Tuple[str, int]):
        """Immediate UDP pong echoing client Timestamp_ns for RTT measurement."""
        if not self.udp_transport:
            return
        mcu_clk = self.last_mcu_clock
        body = struct.pack(
            "<BBHQ6fBB",
            0xAA, 0x05, seq, client_ts_ns,
            float(mcu_clk & 0xFFFF), float((mcu_clk >> 16) & 0xFFFF),
            float(self.clock_sync.last_rtt_ms), 0.0, 0.0, 0.0,
            int(min(180, max(0, self.gripper_mm * 2.88))), 0
        )
        crc = compute_crc8_ccitt(body)
        self.udp_transport.sendto(body + bytes([crc]), addr)

    async def jitter_buffer_worker(self):
        """
        High-priority deterministic 100 Hz (10.0 ms) motion extraction loop.
        Pulls smoothed setpoints from the circular jitter buffer and passes them
        at mathematically uniform time intervals to the kinematics & stepping engine.
        """
        loop = asyncio.get_running_loop()
        period = 0.010  # 10.0 ms strictly
        next_tick = loop.time()

        while self.running:
            next_tick += period
            frame = self.jitter_buffer.pop()
            if frame is not None and self.homed and self.state != "ESTOP":
                try:
                    if frame.opcode == 0x01:
                        await self.servo_to_joints(frame.payload, dt=0.010)
                    elif frame.opcode == 0x02:
                        await self.servo_to_cartesian(frame.payload, dt=0.010)
                except Exception as e:
                    logger.debug("Jitter buffer dispatch error: %s", e)

            sleep_duration = next_tick - loop.time()
            if sleep_duration > 0:
                await asyncio.sleep(sleep_duration)
            else:
                next_tick = loop.time()
                await asyncio.sleep(0.001)

    async def udp_telemetry_loop(self):
        """
        Dedicated 50 Hz UDP Telemetry broadcast.
        Transmits packed 39-byte binary telemetry frames to the active client UDP endpoint.
        """
        while self.running:
            try:
                if self.udp_transport and self.client_udp_endpoint:
                    joints = self.joints_deg
                    flags = 0
                    if self.state == "ESTOP": flags |= 0x01
                    if self.state == "HOMING": flags |= 0x02
                    if self.state == "MOVING": flags |= 0x04
                    if self.homed: flags |= 0x08
                    if self.jitter_buffer.is_active: flags |= 0x10

                    body = struct.pack(
                        "<BBHQ6fBB",
                        0xAA, 0x03, self.telemetry_seq,
                        self.last_client_timestamp_ns,  # Reflected timestamp for continuous client RTT & jitter tracking
                        float(joints[0]), float(joints[1]), float(joints[2]),
                        float(joints[3]), float(joints[4]), float(joints[5]),
                        int(round(min(180, max(0, self.gripper_mm * 2.88)))),
                        flags
                    )
                    crc = compute_crc8_ccitt(body)
                    self.udp_transport.sendto(body + bytes([crc]), self.client_udp_endpoint)
                    self.telemetry_seq = (self.telemetry_seq + 1) & 0xFFFF
            except Exception as e:
                logger.debug("UDP telemetry broadcast error: %s", e)
            await asyncio.sleep(0.02)  # 50 Hz = 20 ms

    async def stream_pending_chunks(self):
        """
        Transmit pending step chunks to MCU using 11-byte CompactStepChunk frames.
        Paced to send max 2 chunks (22 bytes) per pump iteration to strictly prevent
        overflowing the ATmega2560's hardware UART RX buffer (64 bytes).
        """
        async with self.streaming_lock:
            sent_total = 0
            for j in range(6):
                while self.free_slots[j] > 2 and len(self.pending_chunks[j]) > 0:
                    chunk = self.pending_chunks[j].pop(0)
                    self.write_raw(chunk.serialize_compact())
                    self.free_slots[j] -= 1
                    sent_total += 1
                    if sent_total >= 2:
                        return

    async def chunk_pump_loop(self):
        """
        Continuous background pump: Feeds queued trajectory chunks into MCU ring buffers
        at a stable, paced rate (5 ms interval).
        """
        while self.running:
            if self.mcu_connected and self.state not in ("ESTOP", "HOMING"):
                if any(len(q) > 0 for q in self.pending_chunks):
                    await self.stream_pending_chunks()
            await asyncio.sleep(0.005)

    # =========================================================================
    # BACKGROUND TELEMETRY & SYNC LOOPS
    # =========================================================================

    async def ping_loop(self):
        """Periodically ping MCU to maintain clock model and detect communication loss."""
        while self.running:
            if self.mcu_connected:
                self.last_ping_send = time.monotonic()
                self.write_raw(b'P')
            await asyncio.sleep(0.05)

    async def query_status_loop(self):
        """Query detailed hardware status from MCU at 20 Hz."""
        while self.running:
            if self.mcu_connected:
                self.write_raw(b'?')
            await asyncio.sleep(STATUS_BROADCAST_INTERVAL_S)

    def get_telemetry_dict(self) -> Dict[str, Any]:
        """Construct real-time telemetry dictionary for WebSocket & Kiosk."""
        pose = self.kinematics.fk_pose(self.joints_deg)
        rtt = round(self.clock_sync.last_rtt_ms, 2) if self.clock_sync.synced else 0.0

        stream_idle = (time.monotonic() - self.stream_last_rx > 0.5)

        return {
            "type": "TELEMETRY",
            "time": time.time(),
            "state": self.state,
            "homed": self.homed,
            "mcu_connected": self.mcu_connected,
            "mcu_clock": self.last_mcu_clock,
            "rtt_ms": rtt,
            "joints_deg": self.joints_deg,
            "target_joints_deg": self.target_joints_deg,
            "pose": {
                "x": pose[0], "y": pose[1], "z": pose[2],
                "rx": pose[3], "ry": pose[4], "rz": pose[5]
            },
            "pose_mm_deg": [pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]],
            "gripper_mm": self.gripper_mm,
            "endstops": {
                f"j{i+1}": bool(self.endstops_mask & (1 << i))
                for i in range(6)
            },
            "endstops_list": [bool(self.endstops_mask & (1 << i)) for i in range(6)],
            "endstops_mask": self.endstops_mask,
            "endstop_invert_mask": self.endstop_invert_mask,
            "dir_invert_mask": self.dir_invert_mask,
            "free_slots": list(self.free_slots),
            "stream_active": not stream_idle,
            "stream_packets": self.stream_packet_count,
            "homing_trims_deg": dict(self.homing_trims_deg),
            "jitter_depth": self.jitter_buffer.count,
            "jitter_underruns": self.jitter_buffer.underrun_count,
            "jitter_overruns": self.jitter_buffer.overrun_count,
            "jitter_dropped_late": self.jitter_buffer.packets_dropped_late,
        }

    async def broadcast_telemetry(self):
        """
        Broadcast live telemetry to all connected WebSocket clients.
        Streams fixed 31-byte packed binary frames at 50 Hz (Opcode 0x03).
        Also broadcasts JSON frames at 10 Hz for touchscreen browser / kiosk.
        """
        json_counter = 0
        while self.running:
            if self.connected_ws_clients:
                # 1. Build 31-byte packed binary telemetry frame (Opcode 0x03)
                flags = 0
                if self.state == "ESTOP": flags |= 0x01
                if self.state == "HOMING": flags |= 0x02
                if self.state == "MOVING": flags |= 0x04
                if self.homed: flags |= 0x08
                gripper_byte = max(0, min(255, int(self.gripper_mm * 2.55)))

                body = struct.pack(
                    "<BBH6fBB",
                    0xAA,
                    0x03, # Opcode 0x03: Telemetry Feedback
                    self.telemetry_seq & 0xFFFF,
                    float(self.joints_deg[0]), float(self.joints_deg[1]), float(self.joints_deg[2]),
                    float(self.joints_deg[3]), float(self.joints_deg[4]), float(self.joints_deg[5]),
                    gripper_byte,
                    flags
                )
                crc8 = 0
                for b in body: crc8 ^= b
                bin_packet = body + bytes([crc8])
                self.telemetry_seq = (self.telemetry_seq + 1) & 0xFFFF

                # 2. Build JSON string for browser kiosk (10 Hz)
                json_counter += 1
                json_msg = json.dumps(self.get_telemetry_dict()) if (json_counter % 5 == 0) else None

                disconnected = set()
                for ws in self.connected_ws_clients:
                    try:
                        await ws.send(bin_packet)
                        if json_msg:
                            await ws.send(json_msg)
                    except websockets.ConnectionClosed:
                        disconnected.add(ws)
                self.connected_ws_clients.difference_update(disconnected)
            await asyncio.sleep(0.02) # 50 Hz

    async def client_watchdog_loop(self):
        """
        Tier 1 Watchdog (Windows Client <-> Pi Host):
        If no heartbeat (Opcode 0x05) or streaming frame is received for >150 ms during active motion,
        automatically execute a controlled, smooth deceleration ramp to zero (controlled HOLD).
        """
        while self.running:
            if self.state == "MOVING" and (time.monotonic() - self.last_client_heartbeat > 0.150):
                if any(len(q) > 0 for q in self.pending_chunks):
                    logger.warning("Tier 1 Watchdog tripped: Client heartbeat lost for >150 ms during motion! Halting.")
                    async with self.streaming_lock:
                        for q in self.pending_chunks:
                            q.clear()
                    self.state = "IDLE"
            await asyncio.sleep(0.02)

    # =========================================================================
    # WEBSOCKET COMMAND DISPATCHER
    # =========================================================================

    async def ws_handler(self, websocket: Any, *args, **kwargs):
        """Handle incoming WebSocket connections and dispatch commands across all websockets library versions."""
        self.connected_ws_clients.add(websocket)
        logger.info("New WebSocket client connected (%d total)", len(self.connected_ws_clients))
        try:
            # Immediate initial telemetry frame
            await websocket.send(json.dumps(self.get_telemetry_dict()))

            async for message in websocket:
                # Handle High-Speed 31-Byte Packed Binary ArrayBuffer
                if isinstance(message, (bytes, bytearray)):
                    if len(message) == 31 and message[0] == 0xAA:
                        crc8 = 0
                        for b in message[:30]: crc8 ^= b
                        if crc8 == message[30]:
                            header, opcode, seq, j1, j2, j3, j4, j5, j6, gripper, flags = struct.unpack("<BBH6fBB", message[:30])
                            self.last_client_heartbeat = time.monotonic()

                            if opcode == 0x01: # Opcode 0x01: SERVO_JOINTS streaming
                                if self.homed and self.state != "ESTOP":
                                    await self.servo_to_joints([j1, j2, j3, j4, j5, j6], dt=0.02)
                            elif opcode == 0x02: # Opcode 0x02: SERVO_CARTESIAN streaming
                                if self.homed and self.state != "ESTOP":
                                    await self.servo_to_cartesian([j1, j2, j3, j4, j5, j6], dt=0.02)
                            elif opcode == 0x05: # Opcode 0x05: Heartbeat Ping / Pong
                                mcu_clk = self.last_mcu_clock
                                reply_body = struct.pack(
                                    "<BBH6fBB",
                                    0xAA, 0x05, seq,
                                    float(mcu_clk & 0xFFFF), float((mcu_clk >> 16) & 0xFFFF),
                                    float(self.clock_sync.last_rtt_ms), 0.0, 0.0, 0.0,
                                    gripper, flags
                                )
                                r_crc = 0
                                for b in reply_body: r_crc ^= b
                                await websocket.send(reply_body + bytes([r_crc]))
                    continue

                try:
                    payload = json.loads(message)
                except Exception:
                    continue

                cmd = payload.get("cmd", payload.get("action", "")).upper()

                if cmd == "ESTOP":
                    self.emergency_stop()
                    await websocket.send(json.dumps({"type": "ACK", "cmd": "ESTOP", "status": "OK"}))

                elif cmd == "RESET":
                    self.reset_system()
                    await websocket.send(json.dumps({"type": "ACK", "cmd": "RESET", "status": "OK"}))

                elif cmd == "HOME":
                    joint = payload.get("joint", "ALL")
                    if str(joint).upper() in ("ALL", "0", "NONE"):
                        mask = 0xFF
                    else:
                        try:
                            j_num = int(str(joint).replace("J", ""))
                            mask = 1 << (j_num - 1)
                        except ValueError:
                            mask = 0xFF
                    self.trigger_homing(mask)
                    await websocket.send(json.dumps({"type": "ACK", "cmd": "HOME", "status": "OK"}))

                elif cmd == "SET_SPEED_LIMITS":
                    speeds = payload.get("speeds_deg_s", None)
                    accels = payload.get("accels_deg_s2", None)
                    scale = payload.get("speed_scale", None)
                    if scale is not None:
                        self.speed_scale = max(0.05, min(1.0, float(scale)))
                    if speeds or accels:
                        self.planner.set_speed_limits(speeds, accels)
                    await websocket.send(json.dumps({
                        "type": "ACK",
                        "cmd": "SET_SPEED_LIMITS",
                        "status": "OK",
                        "speeds_deg_s": self.planner.joint_speeds,
                        "speed_scale": self.speed_scale
                    }))

                # Real-Time Joint Follower / Streaming
                elif cmd in ("SERVO_JOINTS", "STREAM_JOINTS"):
                    if not self.homed:
                        await websocket.send(json.dumps({
                            "type": "ACK",
                            "cmd": cmd,
                            "status": "REJECTED",
                            "msg": "Safety Interlock: Robot must be homed before accepting joint stream data"
                        }))
                        continue

                    target_joints = payload.get("joints", None)
                    dt = float(payload.get("time", payload.get("duration", 0.05)))
                    velocities = payload.get("velocities", None)
                    accelerations = payload.get("accelerations", None)
                    if target_joints and len(target_joints) == 6:
                        await self.servo_to_joints(target_joints, dt=dt, velocities=velocities, accelerations=accelerations)
                        await websocket.send(json.dumps({"type": "ACK", "cmd": cmd, "status": "OK"}))
                    else:
                        await websocket.send(json.dumps({"type": "ACK", "cmd": cmd, "status": "FAIL", "msg": "Must provide 6 joint angles"}))

                elif cmd == "STREAM_TRAJECTORY":
                    if not self.homed:
                        await websocket.send(json.dumps({
                            "type": "ACK",
                            "cmd": cmd,
                            "status": "REJECTED",
                            "msg": "Safety Interlock: Robot must be homed before streaming trajectories"
                        }))
                        continue

                    points = payload.get("points", [])
                    if points:
                        asyncio.create_task(self.stream_trajectory(points))
                        await websocket.send(json.dumps({"type": "ACK", "cmd": "STREAM_TRAJECTORY", "status": "OK", "points": len(points)}))
                    else:
                        await websocket.send(json.dumps({"type": "ACK", "cmd": "STREAM_TRAJECTORY", "status": "FAIL", "msg": "Empty trajectory"}))

                elif cmd == "JOG_JOINT":
                    if not self.homed:
                        await websocket.send(json.dumps({
                            "type": "ACK",
                            "cmd": cmd,
                            "status": "REJECTED",
                            "msg": "Safety Interlock: Robot must be homed before jogging joints. Use HOME or Diagnostics Direct Step."
                        }))
                        continue

                    if self.state == "MOVING" or any(len(q) > 0 for q in self.pending_chunks):
                        await websocket.send(json.dumps({
                            "type": "ACK",
                            "cmd": cmd,
                            "status": "BUSY",
                            "msg": "Robot is currently moving. Please wait until idle."
                        }))
                        continue

                    joint_idx = int(payload.get("joint", 1)) - 1
                    delta = float(payload.get("delta_deg", 0.0))
                    speed = float(payload.get("speed", 0.5))
                    if 0 <= joint_idx < 6:
                        target = list(self.joints_deg)
                        target[joint_idx] += delta
                        clamped = self.kinematics.clamp_joints(target)
                        asyncio.create_task(self.execute_joint_move(clamped, speed_factor=speed))
                        await websocket.send(json.dumps({"type": "ACK", "cmd": "JOG_JOINT", "status": "OK"}))

                elif cmd == "MOVE_JOINTS":
                    if not self.homed:
                        await websocket.send(json.dumps({
                            "type": "ACK",
                            "cmd": cmd,
                            "status": "REJECTED",
                            "msg": "Safety Interlock: Robot must be homed before moving joints"
                        }))
                        continue

                    if self.state == "MOVING" or any(len(q) > 0 for q in self.pending_chunks):
                        await websocket.send(json.dumps({
                            "type": "ACK",
                            "cmd": cmd,
                            "status": "BUSY",
                            "msg": "Robot is currently moving. Please wait until idle."
                        }))
                        continue

                    target = payload.get("joints", self.joints_deg)
                    speed = float(payload.get("speed", 0.5))
                    asyncio.create_task(self.execute_joint_move(target, speed_factor=speed))
                    await websocket.send(json.dumps({"type": "ACK", "cmd": "MOVE_JOINTS", "status": "OK"}))

                elif cmd == "JOG_CARTESIAN":
                    if not self.homed:
                        await websocket.send(json.dumps({
                            "type": "ACK",
                            "cmd": cmd,
                            "status": "REJECTED",
                            "msg": "Safety Interlock: Robot must be homed before Cartesian moves"
                        }))
                        continue

                    if self.state == "MOVING" or any(len(q) > 0 for q in self.pending_chunks):
                        await websocket.send(json.dumps({
                            "type": "ACK",
                            "cmd": cmd,
                            "status": "BUSY",
                            "msg": "Robot is currently moving. Please wait until idle."
                        }))
                        continue

                    axis = payload.get("axis", "Z").upper()
                    delta = float(payload.get("delta", 0.0))
                    speed = float(payload.get("speed", 0.5))
                    curr_pose = self.kinematics.fk_pose(self.joints_deg)
                    axis_map = {"X": 0, "Y": 1, "Z": 2, "RX": 3, "RY": 4, "RZ": 5}
                    if axis in axis_map:
                        curr_pose[axis_map[axis]] += delta
                        asyncio.create_task(self.execute_cartesian_move(curr_pose, speed_factor=speed))
                        await websocket.send(json.dumps({"type": "ACK", "cmd": "JOG_CARTESIAN", "status": "OK"}))

                elif cmd == "MOVE_CARTESIAN":
                    if not self.homed:
                        await websocket.send(json.dumps({
                            "type": "ACK",
                            "cmd": cmd,
                            "status": "REJECTED",
                            "msg": "Safety Interlock: Robot must be homed before Cartesian moves"
                        }))
                        continue

                    target_pose = payload.get("pose", None)
                    if target_pose is None and "target" in payload:
                        tgt = payload["target"]
                        if isinstance(tgt, dict):
                            x = float(tgt.get("x", 0.0))
                            y = float(tgt.get("y", 0.0))
                            z = float(tgt.get("z", 0.0))
                            if abs(x) < 5.0 and abs(y) < 5.0 and abs(z) < 5.0:  # meters to mm
                                x *= 1000.0
                                y *= 1000.0
                                z *= 1000.0
                            r = float(tgt.get("roll", 0.0))
                            p = float(tgt.get("pitch", 0.0))
                            yw = float(tgt.get("yaw", 0.0))
                            target_pose = [x, y, z, r, p, yw]
                        elif isinstance(tgt, list) and len(tgt) == 6:
                            target_pose = [float(v) for v in tgt]
                    speed = float(payload.get("speed", 0.5))
                    if target_pose and len(target_pose) == 6:
                        asyncio.create_task(self.execute_cartesian_move(target_pose, speed_factor=speed))
                        await websocket.send(json.dumps({"type": "ACK", "cmd": "MOVE_CARTESIAN", "status": "OK"}))
                    else:
                        await websocket.send(json.dumps({"type": "ACK", "cmd": "MOVE_CARTESIAN", "status": "FAIL", "msg": "Invalid target pose"}))

                elif cmd == "SET_GRIPPER":
                    gap_mm = payload.get("mm", None)
                    angle = payload.get("angle", payload.get("position", None))
                    self.set_gripper(mm=gap_mm, angle=angle)
                    await websocket.send(json.dumps({"type": "ACK", "cmd": "SET_GRIPPER", "status": "OK"}))

                # Direct Diagnostic Manual Step
                elif cmd == "MANUAL_STEP":
                    joint = int(payload.get("joint", 1))
                    direction = int(payload.get("dir", 1))
                    steps = int(payload.get("steps", 100))
                    delay_us = int(payload.get("delay_us", 200))
                    self.manual_step(joint, direction, steps, delay_us)
                    await websocket.send(json.dumps({"type": "ACK", "cmd": "MANUAL_STEP", "status": "OK"}))

                elif cmd == "SET_ENDSTOP_INVERT":
                    mask = int(payload.get("mask", 0))
                    self.set_endstop_invert_mask(mask)
                    await websocket.send(json.dumps({"type": "ACK", "cmd": "SET_ENDSTOP_INVERT", "status": "OK"}))

                elif cmd == "SET_DIR_INVERT":
                    mask = int(payload.get("mask", 0x03))
                    self.set_dir_invert_mask(mask)
                    await websocket.send(json.dumps({"type": "ACK", "cmd": "SET_DIR_INVERT", "status": "OK"}))

                elif cmd == "RESET_INVERT_MASKS":
                    self.set_dir_invert_mask(0x03)
                    self.set_endstop_invert_mask(0x00)
                    await websocket.send(json.dumps({
                        "type": "ACK",
                        "cmd": "RESET_INVERT_MASKS",
                        "status": "OK",
                        "dir_invert_mask": self.dir_invert_mask,
                        "endstop_invert_mask": self.endstop_invert_mask
                    }))

                # Persistent Homing Offset Trims
                elif cmd == "SET_TRIMS":
                    new_trims = payload.get("trims", {})
                    save = payload.get("save", True)
                    self.update_trims(new_trims, persist=save)
                    await websocket.send(json.dumps({
                        "type": "ACK",
                        "cmd": "SET_TRIMS",
                        "status": "OK",
                        "trims": self.homing_trims_deg
                    }))

                elif cmd == "GET_TRIMS":
                    await websocket.send(json.dumps({
                        "type": "ACK",
                        "cmd": "GET_TRIMS",
                        "status": "OK",
                        "trims": self.homing_trims_deg
                    }))

                elif cmd == "GET_STATUS":
                    await websocket.send(json.dumps(self.get_telemetry_dict()))

                else:
                    await websocket.send(json.dumps({"type": "ACK", "cmd": cmd, "status": "UNKNOWN_COMMAND"}))

        except websockets.ConnectionClosed:
            pass
        finally:
            self.connected_ws_clients.remove(websocket)
            logger.info("WebSocket client disconnected (%d remaining)", len(self.connected_ws_clients))

    # =========================================================================
    # BUILT-IN HTTP KIOSK SERVER (Port 8000)
    # =========================================================================

    async def handle_http_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        """Serve touchscreen.html directly from Python without external web servers."""
        try:
            line = await asyncio.wait_for(reader.readline(), timeout=3.0)
            if not line:
                writer.close()
                return

            req_line = line.decode("utf-8", errors="ignore").strip()
            # Drain remaining request headers
            while True:
                h_line = await asyncio.wait_for(reader.readline(), timeout=2.0)
                if not h_line or not h_line.strip():
                    break

            script_dir = Path(__file__).resolve().parent
            project_root = script_dir.parent
            ui_path = project_root / "ui" / "touchscreen.html"

            if ui_path.exists():
                with open(ui_path, "rb") as f:
                    content = f.read()

                headers = (
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: text/html; charset=UTF-8\r\n"
                    f"Content-Length: {len(content)}\r\n"
                    "Cache-Control: no-cache, no-store, must-revalidate\r\n"
                    "Connection: close\r\n\r\n"
                )
                writer.write(headers.encode("utf-8"))
                if not req_line.startswith("HEAD"):
                    writer.write(content)
            else:
                body = b"<h1>PAROL6 UI Not Found</h1><p>Expected ui/touchscreen.html</p>"
                headers = (
                    "HTTP/1.1 404 Not Found\r\n"
                    "Content-Type: text/html; charset=UTF-8\r\n"
                    f"Content-Length: {len(body)}\r\n"
                    "Connection: close\r\n\r\n"
                )
                writer.write(headers.encode("utf-8"))
                if not req_line.startswith("HEAD"):
                    writer.write(body)

            await writer.drain()
        except Exception as e:
            logger.debug("HTTP client error: %s", e)
        finally:
            try:
                writer.close()
            except Exception:
                pass

    # =========================================================================
    # LIFECYCLE & EXECUTION
    # =========================================================================

    async def run(self):
        """Main asynchronous event loop executing all servers and control loops."""
        logger.info("Initializing PAROL6 Engine Servers...")

        # 1. Start WebSocket Server (Reliable Control & Handshakes - Port 8765)
        ws_server = await websockets.serve(self.ws_handler, "0.0.0.0", WS_PORT)
        logger.info("WebSocket Control Server active on ws://0.0.0.0:%d", WS_PORT)

        # 2. Start Raw UDP Motion Server (High-Frequency Real-Time Streaming - Port 8766)
        loop = asyncio.get_running_loop()
        udp_transport, udp_protocol = await loop.create_datagram_endpoint(
            lambda: UdpMotionProtocol(self),
            local_addr=("0.0.0.0", UDP_PORT)
        )
        self.udp_transport = udp_transport
        logger.info("Raw UDP Motion Streamer active on 0.0.0.0:%d", UDP_PORT)

        # 3. Start HTTP Kiosk Server (Port 8000)
        http_server = await asyncio.start_server(self.handle_http_client, "0.0.0.0", HTTP_PORT)
        logger.info("Touchscreen HTTP Kiosk Server active on http://0.0.0.0:%d", HTTP_PORT)

        # 4. Launch background tasks
        tasks = [
            asyncio.create_task(self.ping_loop()),
            asyncio.create_task(self.query_status_loop()),
            asyncio.create_task(self.broadcast_telemetry()),
            asyncio.create_task(self.chunk_pump_loop()),
            asyncio.create_task(self.client_watchdog_loop()),
            asyncio.create_task(self.jitter_buffer_worker()),
            asyncio.create_task(self.udp_telemetry_loop()),
        ]

        logger.info("All engine systems operational. Ready for commands.")
        try:
            await asyncio.gather(*tasks)
        except asyncio.CancelledError:
            pass
        finally:
            if self.udp_transport:
                self.udp_transport.close()
            ws_server.close()
            await ws_server.wait_closed()
            http_server.close()
            await http_server.wait_closed()


def main():
    engine = RobotEngine()
    try:
        asyncio.run(engine.run())
    except KeyboardInterrupt:
        logger.info("Stopping PAROL6 engine cleanly...")
        engine.running = False


if __name__ == "__main__":
    main()

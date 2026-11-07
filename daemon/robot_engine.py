#!/usr/bin/env python3
"""
PAROL6 Robotics Engine: 6-DOF Kinematics, Klipper Motion Core & Daemon
======================================================================
Production-grade host control suite for PAROL6 6-DOF robotic arm running on Raspberry Pi 5.
- Architecture: 3-tier headless architecture with 6-Axis Klipper Core.
- Kinematics: Standard DH parameter model + Levenberg-Marquardt (LM) numerical IK with
  analytical spherical wrist invariant branch selection and null-space posture regularization.
- Teleoperation: 1-Euro adaptive filtering (6D) with physical velocity rate limit gating
  and shortest angular distance phase unwrapping.
- Motion Execution: Coordinated 6-axis stream piping directly into Klipper's low-latency
  trapq motion queue (/tmp/printer).
- Transports:
  * Port 8765 (WebSocket): Discrete control, homing, E-stop, trims read/write, telemetry.
  * Port 8766 (UDP): Low-latency 32-byte binary motion streaming @ 50–100 Hz.
  * Port 8000 (HTTP): Touchscreen Kiosk UI serving ui/touchscreen.html.
"""

from __future__ import annotations

import asyncio
from collections import deque
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
import websockets

from kinematics import (
    Parol6Kinematics,
    CAD_HOME_DEG,
    HARDWARE_LIMITS_DEG,
    STEPS_PER_DEG,
    PRESET_POSES_DEG,
    MAX_JOINT_SPEEDS_DEG_S,
    MAX_JOINT_ACCELS_DEG_S2,
    matrix_to_rpy,
)
from one_euro_filter import OneEuroFilter6D
from klipper_bridge import KlipperBridge

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
WS_PORT = 8765
UDP_PORT = 8766
HTTP_PORT = 8000
SERIAL_BAUD = 250000
STATUS_BROADCAST_INTERVAL_S = 0.05  # 20 Hz WebSocket telemetry broadcast

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
    """Compute CRC-8-CCITT (poly 0x07, init 0x00) using lookup table."""
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
    xor_val = 0
    for b in body:
        xor_val ^= b
    return xor_val == expected


# ============================================================================
# CIRCULAR JITTER BUFFER (100 Hz DETERMINISTIC EXTRACTOR)
# ============================================================================
class JitterBufferFrame:
    __slots__ = ("seq", "timestamp_ns", "opcode", "payload", "gripper", "flags", "arrival_mono")

    def __init__(self):
        self.seq = 0
        self.timestamp_ns = 0
        self.opcode = 0
        self.payload: List[float] = [0.0] * 6
        self.gripper = 0
        self.flags = 0
        self.arrival_mono = 0.0

class JitterBuffer:
    """
    Fixed-capacity circular buffer with zero-allocation slot recycling.
    Orders out-of-order UDP motion frames, drops stale duplicates,
    and paces frames at uniform 100 Hz cadence.
    """
    def __init__(self, capacity: int = 64, target_horizon_ms: float = 60.0):
        self.capacity = capacity
        self.target_horizon_s = target_horizon_ms / 1000.0
        self.pool: List[JitterBufferFrame] = [JitterBufferFrame() for _ in range(capacity)]
        self.queue: List[JitterBufferFrame] = []
        self.head_seq = -1
        self.lock = threading.Lock()

        # Telemetry metrics
        self.underruns = 0
        self.overruns = 0
        self.dropped_stale = 0
        self.is_active = False

    @property
    def underrun_count(self) -> int:
        return self.underruns

    @property
    def packets_dropped_overflow(self) -> int:
        return self.overruns

    @property
    def packets_dropped_stale(self) -> int:
        return self.dropped_stale

    def reset(self):
        with self.lock:
            self.queue.clear()
            self.head_seq = -1
            self.is_active = False

    def push(self, seq: int, timestamp_ns: int, opcode: int, payload: List[float], gripper: int, flags: int) -> bool:
        arrival_mono = time.monotonic()
        with self.lock:
            if self.head_seq != -1:
                diff = (seq - self.head_seq) & 0xFFFF
                if diff > 32768:
                    self.dropped_stale += 1
                    return False

            if len(self.queue) >= self.capacity:
                self.queue.pop(0)
                self.overruns += 1

            slot = self.pool[len(self.queue)]
            slot.seq = seq
            slot.timestamp_ns = timestamp_ns
            slot.opcode = opcode
            slot.payload[:] = payload[:6]
            slot.gripper = gripper
            slot.flags = flags
            slot.arrival_mono = arrival_mono

            idx = 0
            while idx < len(self.queue):
                q_diff = (seq - self.queue[idx].seq) & 0xFFFF
                if q_diff > 32768:
                    break
                idx += 1
            self.queue.insert(idx, slot)

            self.head_seq = seq
            self.is_active = True
            return True

    def pop(self) -> Optional[JitterBufferFrame]:
        with self.lock:
            if not self.queue:
                if self.is_active:
                    self.underruns += 1
                return None
            return self.queue.pop(0)


# ============================================================================
# RAW UDP STREAMING PROTOCOL (PORT 8766)
# ============================================================================
class UdpMotionProtocol(asyncio.DatagramProtocol):
    def __init__(self, engine: RobotEngine):
        self.engine = engine

    def connection_made(self, transport: asyncio.DatagramTransport):
        self.engine.udp_transport = transport
        logger.info("Raw UDP Motion socket initialized on 0.0.0.0:%d", UDP_PORT)

    def datagram_received(self, data: bytes, addr: Tuple[str, int]):
        self.engine.handle_udp_packet(data, addr)

    def error_received(self, exc: Exception):
        logger.warning("UDP Motion socket error: %s", exc)

    def connection_lost(self, exc: Optional[Exception]):
        logger.info("UDP Motion socket closed.")


# ============================================================================
# MASTER ROBOT ENGINE (6-AXIS KLIPPER CORE POWERED)
# ============================================================================
class RobotEngine:
    def __init__(self):
        logger.info("Initializing PAROL6 Engine (6-Axis Klipper Stack)...")
        self.running = True

        # Subsystems
        self.kinematics = Parol6Kinematics()
        self.klipper = KlipperBridge(
            pty_path="/tmp/printer",
            fallback_path="/home/mark/printer_data/comms/klippy.serial",
            query_rate_hz=10.0,
        )
        self.comm_logs = deque(maxlen=150)
        self.comm_log_seq: int = 0
        self.klipper.on_comm_log = self.add_comm_log

        # Configurable persistent homing offset trims
        self.load_offsets()

        # State tracking
        self.state: str = "IDLE"  # IDLE, MOVING, HOMING, ESTOP
        self.homed: bool = False
        self.homed_mask: int = 0x00
        self.actual_joints_deg: List[float] = list(CAD_HOME_DEG)
        self.target_joints_deg: List[float] = list(CAD_HOME_DEG)
        self.gripper_mm: float = 62.5
        self.last_gripper_angle: float = 90.0
        self.pending_gripper_angle: Optional[float] = None
        self.last_gripper_tx_time: float = 0.0
        self.last_move_cmd_time: float = 0.0
        self.speed_scale: float = 0.4
        self.telemetry_seq: int = 0
        self.last_client_heartbeat: float = time.monotonic()

        # Streaming & Teleop Conditioning
        self.stream_last_rx: float = 0.0
        self.last_stream_rx_time: float = 0.0
        self.stream_packet_count: int = 0
        self.udp_rx_log_count: int = 0
        self.teleop_filter = OneEuroFilter6D(min_cutoff=3.5, beta=0.25, d_cutoff=1.5)
        self.last_gated_targets: Optional[List[float]] = None
        self.commanded_jog_target: List[float] = list(CAD_HOME_DEG)
        self.smoothed_feedrate: float = 2400.0

        # Diagnostic telemetry
        self.last_raw_target: List[float] = list(CAD_HOME_DEG)
        self.last_ik_joints: List[float] = list(CAD_HOME_DEG)
        self.last_filtered_joints: List[float] = list(CAD_HOME_DEG)

        # Hybrid Transport (UDP Stream + Jitter Buffer)
        self.udp_transport: Optional[asyncio.DatagramTransport] = None
        self.client_udp_endpoint: Optional[Tuple[str, int]] = None
        self.jitter_buffer = JitterBuffer(capacity=64, target_horizon_ms=15.0)
        self.last_client_timestamp_ns: int = 0

        # Connected WebSocket clients
        self.connected_ws_clients: Set[websockets.WebSocketServerProtocol] = set()

    # =========================================================================
    # JOINT POSITIONS & STATE PROPERTIES
    # =========================================================================

    def add_comm_log(self, link: str, direction: str, msg: str):
        """Record a dual-link RX/TX/ERR communication entry for Suite <-> RPi and RPi <-> MCU."""
        self.comm_log_seq += 1
        now = time.time()
        ts = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int((now % 1) * 1000):03d}"
        self.comm_logs.append({
            "id": self.comm_log_seq,
            "ts": ts,
            "link": link,      # "SUITE_RPI" or "RPI_MCU"
            "dir": direction,  # "RX", "TX", or "ERR"
            "msg": str(msg),
        })

    def _sync_homed_from_klipper(self):
        """Synchronize daemon homed flag if Klipper core is already homed."""
        if not self.homed and self.klipper.is_homed:
            self.homed = True
            self.homed_mask = self.klipper.homed_mask or 0x3F
            live = list(self.klipper.actual_joints_deg)
            self.actual_joints_deg = list(live)
            self.target_joints_deg = list(live)
            self.commanded_jog_target = list(live)
            self.teleop_filter.reset(live)
            logger.info("Synchronized homed state from Klipper core: joints=%s", live)

    @property
    def joints_deg(self) -> List[float]:
        """Live physical joint angles from Klipper M114 telemetry."""
        if self.klipper.is_ready:
            return list(self.klipper.actual_joints_deg)
        return list(self.actual_joints_deg)

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
        """Update live homing trims and persist to offsets.json."""
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
                logger.info("Persisted updated homing trims: %s", self.homing_trims_deg)
            except Exception as e:
                logger.error("Failed saving offsets.json: %s", e)

    # =========================================================================
    # CORE ROBOT ACTIONS (KLIPPER DELEGATED)
    # =========================================================================

    def emergency_stop(self):
        """Engage hardware E-Stop immediately via Klipper M112."""
        logger.critical("EMERGENCY STOP ENGAGED!")
        self.state = "ESTOP"
        self.klipper.emergency_stop()
        self.jitter_buffer.reset()

    def reset_system(self):
        """Reset MCU and reload Klipper firmware via FIRMWARE_RESTART."""
        logger.info("Resetting system via Klipper FIRMWARE_RESTART...")
        self.klipper.firmware_restart()
        self.state = "IDLE"
        self.jitter_buffer.reset()

    def trigger_homing(self, mask: int = 0xFF):
        """
        Trigger asynchronous full PAROL6 homing sequence through Klipper macro (HOME).
        """
        if self.state == "HOMING":
            logger.warning("Homing already in progress, ignoring duplicate request.")
            return
        asyncio.create_task(self._async_homing(mask=mask))

    async def _async_homing(self, mask: int = 0xFF):
        logger.info("Executing PAROL6 homing sequence via Klipper...")
        self.state = "HOMING"
        self.homed = False
        self.homed_mask = 0x00

        try:
            success = await asyncio.to_thread(self.klipper.home_all, 120.0)
            if success:
                # Apply persistent CAD reference + degree calibration trims
                calibrated_targets = [CAD_HOME_DEG[i] + self.homing_trims_array[i] for i in range(6)]
                self.klipper.set_kinematic_position(*calibrated_targets)
                self.target_joints_deg = list(calibrated_targets)
                self.actual_joints_deg = list(calibrated_targets)
                self.teleop_filter.reset(calibrated_targets)
                self.last_gated_targets = None
                self.homed = True
                self.homed_mask = 0x3F
                self.state = "IDLE"
                logger.info("PAROL6 Homing complete. Joint positions calibrated to CAD home: %s", calibrated_targets)
            else:
                logger.error("Homing sequence failed or timed out!")
                self.state = "IDLE"
        except Exception as e:
            logger.error("Error during homing sequence: %s", e)
            self.state = "IDLE"

    def set_gripper(self, mm: Optional[float] = None, angle: Optional[float] = None, force: bool = False):
        """
        Control gripper opening via Klipper servo module (prioritizes direct angle).
        Enforces Stationary Interlock (queues command while arm is MOVING/HOMING)
        and rate-limits SET_SERVO G-code to prevent flooding Klipper's AVR software PWM timer.
        """
        if angle is not None:
            ang = max(0.0, min(180.0, float(angle)))
        elif mm is not None:
            gap = max(0.0, min(62.5, float(mm)))
            ang = 180.0 * (1.0 - (gap / 62.5))
        else:
            return

        self.last_gripper_angle = ang
        self.gripper_mm = 62.5 * (1.0 - (ang / 180.0))

        # Stationary Interlock: stage gripper target if the robot is actively moving or homing
        if self.state in ("MOVING", "HOMING") and not force:
            self.pending_gripper_angle = ang
            return

        now = time.monotonic()
        if force or (now - self.last_gripper_tx_time >= 0.040) or ang in (0.0, 90.0, 180.0):
            self.last_gripper_tx_time = now
            self.pending_gripper_angle = None
            self.klipper.set_gripper(angle=ang)
        else:
            self.pending_gripper_angle = ang

    async def execute_joint_move(self, target_joints_deg: List[float], speed_factor: float = 0.5) -> bool:
        """Execute a coordinated trapezoidal multi-axis joint move via Klipper."""
        self._sync_homed_from_klipper()
        if not self.homed:
            logger.warning("Joint move rejected: Robot must be homed first!")
            return False

        clamped = self.kinematics.clamp_joints(target_joints_deg)
        spd = max(0.1, min(1.0, float(speed_factor))) * self.speed_scale
        feedrate = max(300.0, 3000.0 * spd)

        cmd = (
            f"G1 X{clamped[0]:.3f} Y{clamped[1]:.3f} Z{clamped[2]:.3f} "
            f"A{clamped[3]:.3f} B{clamped[4]:.3f} C{clamped[5]:.3f} F{feedrate:.1f}"
        )
        self.state = "MOVING"
        self.last_move_cmd_time = time.monotonic()
        success = self.klipper.send_gcode(cmd, log_cmd=True)
        self.target_joints_deg = list(clamped)
        self.commanded_jog_target = list(clamped)
        self.last_gated_targets = list(clamped)
        self.teleop_filter.reset(clamped)
        return success

    async def execute_cartesian_move(self, target_pose: List[float], speed_factor: float = 0.5) -> bool:
        """
        Execute a linear Cartesian move while strictly preserving the 3x3 rotation matrix
        and enforcing deterministic wrist branch locking & 3.0° delta spike protection across every sub-step.
        """
        self._sync_homed_from_klipper()
        if not self.homed:
            logger.warning("Cartesian move rejected: Robot must be homed first!")
            return False

        seed = list(self.target_joints_deg if self.state == "MOVING" else self.joints_deg)
        start_pose = self.kinematics.fk_pose(seed)
        dx = float(target_pose[0]) - float(start_pose[0])
        dy = float(target_pose[1]) - float(start_pose[1])
        dz = float(target_pose[2]) - float(start_pose[2])
        linear_dist_mm = math.sqrt(dx * dx + dy * dy + dz * dz)

        num_steps = max(1, min(160, int(math.ceil(linear_dist_mm / 1.5))))
        curr_q = list(seed)
        waypoints: List[List[float]] = []

        for step_idx in range(1, num_steps + 1):
            alpha = step_idx / float(num_steps)
            step_pose = [
                float(start_pose[0]) + dx * alpha,
                float(start_pose[1]) + dy * alpha,
                float(start_pose[2]) + dz * alpha,
                float(target_pose[3]),
                float(target_pose[4]),
                float(target_pose[5]),
            ]
            ik_res = self.kinematics.solve_ik(step_pose, seed_joints_deg=curr_q)
            if not ik_res.success:
                logger.warning("IK Failed at sub-step %d/%d for Cartesian target %s: %s", step_idx, num_steps, step_pose, ik_res.message)
                return False

            ok_branch, locked_q, reason = self.kinematics.validate_and_lock_branch(
                ik_res.joints_deg, curr_q, max_delta_deg=3.0
            )
            if not ok_branch:
                logger.warning("Cartesian move aborted by safety gate at sub-step %d/%d: %s", step_idx, num_steps, reason)
                self.add_comm_log("SUITE_RPI", "ERR", f"CARTESIAN SAFETY GATE: {reason}")
                return False

            curr_q = list(locked_q)
            waypoints.append(curr_q)

        self.last_raw_target = list(target_pose)
        self.last_ik_joints = list(curr_q)

        spd = max(0.1, min(1.0, float(speed_factor))) * self.speed_scale
        feedrate = max(300.0, 3000.0 * spd)
        self.state = "MOVING"
        self.last_move_cmd_time = time.monotonic()

        success = True
        for wp in waypoints:
            cmd = (
                f"G1 X{wp[0]:.3f} Y{wp[1]:.3f} Z{wp[2]:.3f} "
                f"A{wp[3]:.3f} B{wp[4]:.3f} C{wp[5]:.3f} F{feedrate:.1f}"
            )
            if not self.klipper.send_gcode(cmd, log_cmd=(wp is waypoints[-1])):
                success = False
                break

        self.target_joints_deg = list(curr_q)
        self.commanded_jog_target = list(curr_q)
        self.last_gated_targets = list(curr_q)
        self.teleop_filter.reset(curr_q)
        return success

    async def jog_joint(self, joint_idx: int, direction: int, step_deg: float = 2.0, speed_factor: float = 0.5) -> bool:
        """Jog a single joint by step_deg degrees."""
        self._sync_homed_from_klipper()
        if not self.homed:
            logger.warning("Jog rejected: Robot must be homed first!")
            return False

        target = list(self.target_joints_deg if self.state == "MOVING" else self.joints_deg)
        if 0 <= joint_idx < 6:
            target[joint_idx] += step_deg * (1 if direction > 0 else -1)
            clamped = self.kinematics.clamp_joints(target)
            return await self.execute_joint_move(clamped, speed_factor=speed_factor)
        return False

    async def jog_cartesian(self, axis_idx: int, direction: int, step_size_mm: float = 5.0, speed_factor: float = 0.5) -> bool:
        """Jog a Cartesian end-effector axis (X, Y, Z, Rx, Ry, Rz) using gimbal-lock-free SE(3) increments."""
        self._sync_homed_from_klipper()
        if not self.homed:
            logger.warning("Cartesian jog rejected: Robot must be homed first!")
            return False

        ref_joints = self.target_joints_deg if self.state == "MOVING" else self.joints_deg
        current_pose = self.kinematics.fk_pose(ref_joints)
        target_pose = list(current_pose)
        step = step_size_mm * (1 if direction > 0 else -1)
        if 0 <= axis_idx < 3:
            target_pose[axis_idx] += step
            return await self.execute_cartesian_move(target_pose, speed_factor=speed_factor)
        elif 3 <= axis_idx < 6:
            T_curr = self.kinematics.fk_matrix(ref_joints)
            R_curr = T_curr[:3, :3]
            rad = math.radians(step)
            c, s = math.cos(rad), math.sin(rad)
            if axis_idx == 3:  # World RX
                R_step = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)
            elif axis_idx == 4:  # World RY
                R_step = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)
            else:  # World RZ
                R_step = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
            R_new = R_step @ R_curr
            rx, ry, rz = matrix_to_rpy(R_new)
            target_pose[3] = round(math.degrees(rx), 3)
            target_pose[4] = round(math.degrees(ry), 3)
            target_pose[5] = round(math.degrees(rz), 3)
            return await self.execute_cartesian_move(target_pose, speed_factor=speed_factor)
        return False

    # =========================================================================
    # REAL-TIME TELEOPERATION CONDITIONING & KLIPPER STREAMING
    # =========================================================================

    async def servo_to_joints(
        self,
        target_joints_deg: List[float],
        dt: float = 0.010,
        velocities: Optional[List[float]] = None,
        accelerations: Optional[List[float]] = None,
        is_continuation: bool = False,
    ) -> bool:
        """
        High-rate teleoperation conditioning pipeline (50–100 Hz).
        1. Soft limit clamping.
        2. Shortest Angular Distance phase unwrapping.
        3. Physical velocity rate limiter gate (smoothly slews large steps without deadlocking).
        4. 6-Axis Adaptive 1-Euro Filter.
        5. Synchronous G-code streaming into Klipper's low-latency motion queue.
        """
        if self.state == "ESTOP":
            return False

        self._sync_homed_from_klipper()
        if not self.homed:
            logger.warning("Joint stream rejected: Robot must be homed first!")
            return False

        now_mono = time.monotonic()
        if not is_continuation:
            self.stream_last_rx = now_mono
            self.last_stream_rx_time = now_mono
            self.stream_packet_count += 1

        # 1. Soft joint limit clamping
        clamped_incoming = self.kinematics.clamp_joints(target_joints_deg)
        self.commanded_jog_target = list(clamped_incoming)
        dt_clamped = max(0.005, min(0.2, float(dt)))

        # 2. Shortest Angular Distance Unwrapping
        ref_unwrapped = self.target_joints_deg if self.state == "MOVING" else self.joints_deg
        unwrapped_incoming = list(clamped_incoming)
        for j in range(6):
            delta = math.atan2(
                math.sin(math.radians(clamped_incoming[j] - ref_unwrapped[j])),
                math.cos(math.radians(clamped_incoming[j] - ref_unwrapped[j]))
            )
            unwrapped_incoming[j] = ref_unwrapped[j] + math.degrees(delta)
        unwrapped_incoming = self.kinematics.clamp_joints(unwrapped_incoming)

        # Ground teleop filter if stream was inactive or uninitialized
        if (self.last_gated_targets is None) or (self.teleop_filter.filters[0].last_x is None):
            self.last_gated_targets = list(ref_unwrapped)
            self.teleop_filter.reset(ref_unwrapped)

        # 2b. Pre-filter IK Branch Flip & Singularity Jump Interceptor:
        # Reject any streaming packet that commands a sudden wrist flip / jump (> 12° in a single packet)
        if not is_continuation:
            ok_raw, _, raw_reason = self.kinematics.validate_and_lock_branch(
                unwrapped_incoming, self.last_gated_targets, max_delta_deg=12.0
            )
            if not ok_raw:
                logger.warning("SAFETY INTERCEPTOR rejected streaming frame: %s", raw_reason)
                self.add_comm_log("SUITE_RPI", "ERR", f"SAFETY GATE: {raw_reason}")
                self.commanded_jog_target = list(self.last_gated_targets)
                return False

        # 3. Synchronized 6-Axis Velocity & 3.0°/10ms Delta Rate Limiting Gate
        # Scales all 6 axes proportionally so multi-axis orientation coordination is strictly preserved
        sync_scale = 1.0
        for j in range(6):
            raw_delta_deg = abs(unwrapped_incoming[j] - self.last_gated_targets[j])
            omega_limit_deg_s = MAX_JOINT_SPEEDS_DEG_S[j] * max(0.2, self.speed_scale)
            max_step_j = min(3.0, omega_limit_deg_s * dt_clamped)
            if raw_delta_deg > max_step_j and raw_delta_deg > 1e-5:
                sync_scale = min(sync_scale, max_step_j / raw_delta_deg)

        candidate_gated = [
            self.last_gated_targets[j] + (unwrapped_incoming[j] - self.last_gated_targets[j]) * sync_scale
            for j in range(6)
        ]

        # Enforce strict 3.0° per 10ms window & mechanical cable protection gate
        ok_gate, gated_targets, gate_reason = self.kinematics.validate_and_lock_branch(
            candidate_gated, self.last_gated_targets, max_delta_deg=3.0
        )
        if not ok_gate:
            logger.warning("validate_and_lock_branch rejected step: %s", gate_reason)
            self.add_comm_log("SUITE_RPI", "ERR", f"BRANCH LOCK: {gate_reason}")
            return False

        self.last_gated_targets = list(gated_targets)

        # 4. 6-Axis Adaptive 1-Euro Filter
        filtered_targets = self.teleop_filter.filter(gated_targets, dt_clamped)
        self.last_filtered_joints = list(filtered_targets)

        # 5. Stream Coordinated Move to 6-Axis Klipper Core (skip redundant zero-delta packets)
        max_delta = max(abs(filtered_targets[j] - ref_unwrapped[j]) for j in range(6))
        if max_delta < 0.015:
            return True

        seg_dist_6d = math.sqrt(sum((filtered_targets[j] - ref_unwrapped[j]) ** 2 for j in range(6)))
        self.target_joints_deg = list(filtered_targets)
        self.state = "MOVING"
        self.last_move_cmd_time = now_mono
        raw_feedrate = max(600.0, min(12000.0, (seg_dist_6d / dt_clamped) * 60.0 * 1.15))
        self.smoothed_feedrate = 0.70 * self.smoothed_feedrate + 0.30 * raw_feedrate
        feedrate_deg_min = max(600.0, min(12000.0, self.smoothed_feedrate))
        return self.klipper.stream_target(
            filtered_targets[0],
            filtered_targets[1],
            filtered_targets[2],
            filtered_targets[3],
            filtered_targets[4],
            filtered_targets[5],
            feedrate_deg_min=feedrate_deg_min,
        )

    async def servo_to_cartesian(self, target_pose: List[float], dt: float = 0.010) -> bool:
        """Solve DLS/LM IK for Cartesian streaming setpoint, then pass to servo_to_joints."""
        self._sync_homed_from_klipper()
        if not self.homed:
            return False
        seed = self.target_joints_deg if self.state == "MOVING" else self.joints_deg
        ik_res = self.kinematics.solve_ik(target_pose, seed_joints_deg=seed)
        self.last_raw_target = list(target_pose)
        if not ik_res.success:
            logger.debug("Cartesian streaming IK failure: %s", ik_res.message)
            return False
        self.last_ik_joints = list(ik_res.joints_deg)
        return await self.servo_to_joints(ik_res.joints_deg, dt=dt)

    # =========================================================================
    # UDP STREAMING & JITTER BUFFER EXTRACTION (PORT 8766)
    # =========================================================================

    def handle_udp_packet(self, data: bytes, addr: Tuple[str, int]):
        """Zero-allocation deserializer for high-frequency UDP motion frames (Port 8766)."""
        length = len(data)
        if length < 31 or data[0] != 0xAA:
            return
        if not verify_crc(data):
            logger.debug("UDP packet from %s failed CRC check (len=%d)", addr, length)
            return

        self.client_udp_endpoint = addr
        self._sync_homed_from_klipper()

        try:
            if length == 39:
                header, opcode, seq, ts_ns, j1, j2, j3, j4, j5, j6, gripper, flags, _crc = struct.unpack_from("<BBHQ6fBBB", data)
            elif length == 31:
                header, opcode, seq, j1, j2, j3, j4, j5, j6, gripper, flags, _crc = struct.unpack_from("<BBH6fBBB", data)
                ts_ns = time.perf_counter_ns()
            elif length == 32:
                header, opcode, seq, j1, j2, j3, j4, j5, j6, gripper, flags = struct.unpack_from("<BBH6fBB", data[:30])
                ts_ns = time.perf_counter_ns()
            else:
                return
        except (struct.error, ValueError):
            return

        self.last_client_heartbeat = time.monotonic()
        self.last_client_timestamp_ns = ts_ns

        self.udp_rx_log_count += 1
        if self.udp_rx_log_count == 1 or self.udp_rx_log_count % 100 == 0:
            target_type = "JOINT" if opcode == 0x01 else ("CARTESIAN" if opcode == 0x02 else f"OP_0x{opcode:02X}")
            logger.info(
                "UDP RX #%d from %s: len=%dB, CRC=OK, type=%s, seq=%d, payload=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f], grip=%d",
                self.udp_rx_log_count, addr, length, target_type, seq, j1, j2, j3, j4, j5, j6, gripper
            )

        if opcode in (0x01, 0x02):
            if self.udp_rx_log_count % 15 == 1:
                op_name = "SERVO_JOINTS" if opcode == 0x01 else "SERVO_CARTESIAN"
                self.add_comm_log(
                    "SUITE_RPI",
                    "RX",
                    f"UDP #{seq} {op_name} [{j1:.1f}, {j2:.1f}, {j3:.1f}, {j4:.1f}, {j5:.1f}, {j6:.1f}] grip={gripper}"
                )

        if opcode == 0x01:  # MOVE_JOINTS
            if self.homed and self.state != "ESTOP":
                self.jitter_buffer.push(seq, ts_ns, opcode, [j1, j2, j3, j4, j5, j6], gripper, flags)
            elif not self.homed and self.udp_rx_log_count % 30 == 1:
                self.add_comm_log("SUITE_RPI", "ERR", f"UDP MOVE_JOINTS #{seq} dropped: Robot not homed")
        elif opcode == 0x02:  # MOVE_CARTESIAN
            if self.homed and self.state != "ESTOP":
                self.jitter_buffer.push(seq, ts_ns, opcode, [j1, j2, j3, j4, j5, j6], gripper, flags)
            elif not self.homed and self.udp_rx_log_count % 30 == 1:
                self.add_comm_log("SUITE_RPI", "ERR", f"UDP MOVE_CARTESIAN #{seq} dropped: Robot not homed")
        elif opcode == 0x05:  # HEARTBEAT
            self.send_udp_heartbeat_reply(seq, ts_ns, addr)

    def send_udp_heartbeat_reply(self, seq: int, client_ts_ns: int, addr: Tuple[str, int]):
        """Immediate UDP pong echoing client Timestamp_ns for RTT measurement."""
        if not self.udp_transport:
            return
        body = struct.pack(
            "<BBHQ6fBB",
            0xAA, 0x05, seq, client_ts_ns,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            int(min(180, max(0, self.gripper_mm * 2.88))), 0
        )
        crc = compute_crc8_ccitt(body)
        self.udp_transport.sendto(body + bytes([crc]), addr)

    async def jitter_buffer_worker(self):
        """
        High-priority deterministic 100 Hz (10.0 ms) motion extraction loop.
        Extracts frames from the circular jitter buffer and routes to Klipper.
        Executes controlled soft-stop deceleration if streaming drops for > 60 ms.
        """
        loop = asyncio.get_running_loop()
        period = 0.010
        next_tick = loop.time()

        while self.running:
            next_tick += period
            t_now = time.monotonic()
            frame = self.jitter_buffer.pop()

            if frame is not None and self.homed and self.state != "ESTOP":
                try:
                    if frame.opcode == 0x01:
                        await self.servo_to_joints(frame.payload, dt=0.010)
                    elif frame.opcode == 0x02:
                        await self.servo_to_cartesian(frame.payload, dt=0.010)
                except Exception as e:
                    logger.debug("Jitter buffer dispatch error: %s", e)
            elif self.homed and self.state != "ESTOP":
                dt_since_rx = t_now - self.stream_last_rx
                remaining_err = max(abs(self.commanded_jog_target[j] - self.target_joints_deg[j]) for j in range(6))
                if self.state == "MOVING" and dt_since_rx <= 0.250 and remaining_err > 0.02:
                    # Continue smooth rate-limited slewing toward latest streamed target between packets
                    await self.servo_to_joints(self.commanded_jog_target, dt=0.010, is_continuation=True)
                elif self.state == "MOVING" and dt_since_rx > 0.250:
                    derivatives = self.teleop_filter.get_derivatives()
                    any_moving = any(abs(d) > 0.1 for d in derivatives)

                    if any_moving:
                        dt_stop = 0.010
                        decel_targets = list(self.target_joints_deg)
                        still_active = False

                        for j in range(6):
                            v_j = derivatives[j]
                            if abs(v_j) > 0.1:
                                a_decel = max(MAX_JOINT_ACCELS_DEG_S2[j] * self.speed_scale, abs(v_j) / 0.08)
                                new_speed = max(0.0, abs(v_j) - a_decel * dt_stop)
                                if new_speed > 0.1:
                                    v_new = math.copysign(new_speed, v_j)
                                    still_active = True
                                else:
                                    v_new = 0.0
                                decel_targets[j] += v_new * dt_stop
                                self.teleop_filter.filters[j].last_dx = v_new
                            else:
                                self.teleop_filter.filters[j].last_dx = 0.0

                        if still_active:
                            await self.servo_to_joints(decel_targets, dt=dt_stop, is_continuation=True)
                        else:
                            self.teleop_filter.reset(self.target_joints_deg)
                            self.last_gated_targets = None
                            if (t_now - self.last_move_cmd_time > 0.250):
                                live_err = max(abs(self.joints_deg[j] - self.target_joints_deg[j]) for j in range(6))
                                if live_err < 0.6 or (t_now - self.last_move_cmd_time > 2.5):
                                    self.state = "IDLE"
                    else:
                        if (t_now - self.last_move_cmd_time > 0.250):
                            live_err = max(abs(self.joints_deg[j] - self.target_joints_deg[j]) for j in range(6))
                            if live_err < 0.6 or (t_now - self.last_move_cmd_time > 2.5):
                                self.state = "IDLE"

            # Flush any pending gripper angle once stationary (Stationary Interlock + Coalesced Slider Tail)
            if self.state == "IDLE" and self.pending_gripper_angle is not None and (t_now - self.last_gripper_tx_time >= 0.040):
                pending_ang = self.pending_gripper_angle
                self.pending_gripper_angle = None
                self.last_gripper_tx_time = t_now
                self.klipper.set_gripper(angle=pending_ang)

            sleep_duration = next_tick - loop.time()
            if sleep_duration > 0:
                await asyncio.sleep(sleep_duration)
            else:
                next_tick = loop.time()
                await asyncio.sleep(0.001)

    async def udp_telemetry_loop(self):
        """Dedicated 50 Hz UDP Telemetry broadcast."""
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
                        self.last_client_timestamp_ns,
                        float(joints[0]), float(joints[1]), float(joints[2]),
                        float(joints[3]), float(joints[4]), float(joints[5]),
                        int(round(min(180, max(0, self.gripper_mm * 2.88)))),
                        flags
                    )
                    crc = compute_crc8_ccitt(body)
                    self.udp_transport.sendto(body + bytes([crc]), self.client_udp_endpoint)
                    self.telemetry_seq = (self.telemetry_seq + 1) & 0xFFFF
            except Exception as e:
                logger.debug("UDP Telemetry error: %s", e)
            await asyncio.sleep(0.02)

    # =========================================================================
    # WEBSOCKET CONTROL & TELEMETRY (PORT 8765)
    # =========================================================================

    def get_telemetry_dict(self) -> Dict[str, Any]:
        """Generate full JSON telemetry dictionary for WebSocket clients."""
        self._sync_homed_from_klipper()
        joints = self.joints_deg
        pose = self.kinematics.fk_pose(joints)
        return {
            "type": "TELEMETRY",
            "seq": self.telemetry_seq,
            "timestamp": time.time(),
            "state": self.state,
            "homed": self.homed,
            "homed_mask": self.klipper.homed_mask if self.klipper.is_connected else 0,
            "joints_deg": [round(float(v), 2) for v in joints],
            "target_joints_deg": [round(float(v), 2) for v in self.target_joints_deg],
            "pose": {
                "x": round(float(pose[0]), 2),
                "y": round(float(pose[1]), 2),
                "z": round(float(pose[2]), 2),
                "roll": round(float(pose[3]), 2),
                "pitch": round(float(pose[4]), 2),
                "yaw": round(float(pose[5]), 2),
            },
            "gripper_mm": round(self.gripper_mm, 1),
            "gripper_deg": round(self.last_gripper_angle, 1),
            "mcu_connected": self.klipper.is_ready,
            "klipper_connected": self.klipper.is_connected,
            "klipper_ready": self.klipper.is_ready,
            "rtt_ms": 1.0,
            "speed_scale": self.speed_scale,
            "homing_trims_deg": self.homing_trims_deg,
            "streaming_active": (self.state == "MOVING" and (time.monotonic() - self.last_stream_rx_time < 0.25)),
            "last_mcu_error": self.klipper.last_error,
            "comm_logs": list(self.comm_logs)[-50:],
        }

    async def broadcast_telemetry(self):
        """20 Hz periodic WebSocket telemetry broadcast."""
        while self.running:
            if self.connected_ws_clients:
                msg = json.dumps(self.get_telemetry_dict())
                disconnected = set()
                for ws in self.connected_ws_clients:
                    try:
                        await ws.send(msg)
                    except Exception:
                        disconnected.add(ws)
                for ws in disconnected:
                    self.connected_ws_clients.discard(ws)
            await asyncio.sleep(STATUS_BROADCAST_INTERVAL_S)

    async def process_command(self, payload: dict) -> dict:
        """Central command dispatcher for WebSocket and REST API."""
        cmd = payload.get("cmd", payload.get("action", "")).upper()

        if cmd == "PING":
            return {
                "type": "PONG",
                "client_time": payload.get("time", time.time()),
                "server_time": time.time(),
            }

        elif cmd in ("GET_TELEMETRY", "STATUS"):
            return self.get_telemetry_dict()

        elif cmd == "CLEAR_LOGS":
            self.comm_logs.clear()
            self.klipper.last_error = ""
            return {"type": "ACK", "cmd": "CLEAR_LOGS", "status": "OK"}

        # Log incoming control command on SUITE <-> RPI channel
        self.add_comm_log("SUITE_RPI", "RX", json.dumps(payload))
        resp = await self._dispatch_logged_command(cmd, payload)
        if resp:
            status = resp.get("status", "OK")
            direction = "ERR" if status in ("ERROR", "REJECTED", "UNKNOWN_COMMAND") else "TX"
            self.add_comm_log("SUITE_RPI", direction, json.dumps(resp))
        return resp

    async def _dispatch_logged_command(self, cmd: str, payload: dict) -> dict:
        """Execute command and return response dictionary."""
        if cmd in ("ESTOP", "EMERGENCY_STOP"):
            self.emergency_stop()
            return {"type": "ACK", "cmd": "ESTOP", "status": "OK"}

        elif cmd in ("RESET", "CLEAR_ESTOP", "RECONNECT_MCU"):
            self.reset_system()
            return {"type": "ACK", "cmd": cmd, "status": "OK"}

        elif cmd == "HOME":
            mask = payload.get("mask", 0xFF)
            self.trigger_homing(mask=mask)
            return {"type": "ACK", "cmd": "HOME", "status": "OK"}

        elif cmd == "HOME_JOINT":
            joint_idx = int(payload.get("joint", 1)) - 1
            self.klipper.home_joint(joint_idx)
            return {"type": "ACK", "cmd": "HOME_JOINT", "status": "OK", "joint": joint_idx + 1}

        elif cmd in ("SET_CAD_HOME", "GO_CAD_HOME"):
            calibrated_targets = [CAD_HOME_DEG[i] + self.homing_trims_array[i] for i in range(6)]
            self.klipper.set_kinematic_position(*calibrated_targets)
            self.target_joints_deg = list(calibrated_targets)
            self.actual_joints_deg = list(calibrated_targets)
            self.teleop_filter.reset(calibrated_targets)
            self.last_gated_targets = None
            self.homed = True
            self.homed_mask = 0x3F
            logger.info("Executed SET_CAD_HOME: Joint positions forced to CAD zero: %s", calibrated_targets)
            return {"type": "ACK", "cmd": "SET_CAD_HOME", "status": "OK", "joints": calibrated_targets}

        elif cmd in ("WAKE_ARM", "ENERGIZE"):
            self.klipper.send_gcode("WAKE_ARM", log_cmd=True)
            return {"type": "ACK", "cmd": "WAKE_ARM", "status": "OK"}

        elif cmd == "SET_SPEED_SCALE":
            scale = float(payload.get("scale", payload.get("value", 0.4)))
            self.speed_scale = max(0.1, min(1.0, scale))
            return {"type": "ACK", "cmd": "SET_SPEED_SCALE", "status": "OK", "scale": self.speed_scale}

        elif cmd in ("SERVO_JOINTS", "STREAM_JOINTS"):
            self._sync_homed_from_klipper()
            if not self.homed:
                return {"type": "ACK", "cmd": cmd, "status": "REJECTED", "msg": "Robot must be homed first"}
            target_joints = payload.get("joints", None)
            dt = float(payload.get("time", payload.get("duration", 0.05)))
            if target_joints and len(target_joints) == 6:
                await self.servo_to_joints(target_joints, dt=dt)
                return {"type": "ACK", "cmd": cmd, "status": "OK"}
            return {"type": "ACK", "cmd": cmd, "status": "ERROR", "msg": "Invalid joints array"}

        elif cmd == "MOVE_JOINTS":
            self._sync_homed_from_klipper()
            if not self.homed:
                return {"type": "ACK", "cmd": cmd, "status": "REJECTED", "msg": "Robot must be homed first"}
            target = payload.get("joints", self.joints_deg)
            speed = float(payload.get("speed", 0.5))
            asyncio.create_task(self.execute_joint_move(target, speed_factor=speed))
            return {"type": "ACK", "cmd": "MOVE_JOINTS", "status": "OK"}

        elif cmd == "JOG_JOINT":
            self._sync_homed_from_klipper()
            if not self.homed:
                return {"type": "ACK", "cmd": cmd, "status": "REJECTED", "msg": "Robot must be homed first"}
            joint_idx = int(payload.get("joint", 1)) - 1
            delta = float(payload.get("delta_deg", 0.0))
            speed = float(payload.get("speed", 0.5))
            asyncio.create_task(self.jog_joint(joint_idx, 1 if delta > 0 else -1, step_deg=abs(delta), speed_factor=speed))
            return {"type": "ACK", "cmd": "JOG_JOINT", "status": "OK"}

        elif cmd == "MOVE_CARTESIAN":
            self._sync_homed_from_klipper()
            if not self.homed:
                return {"type": "ACK", "cmd": cmd, "status": "REJECTED", "msg": "Robot must be homed first"}
            target_pose = payload.get("pose", None)
            speed = float(payload.get("speed", 0.5))
            if target_pose and len(target_pose) == 6:
                asyncio.create_task(self.execute_cartesian_move(target_pose, speed_factor=speed))
                return {"type": "ACK", "cmd": "MOVE_CARTESIAN", "status": "OK"}
            return {"type": "ACK", "cmd": "MOVE_CARTESIAN", "status": "ERROR", "msg": "Invalid pose array"}

        elif cmd == "JOG_CARTESIAN":
            self._sync_homed_from_klipper()
            if not self.homed:
                return {"type": "ACK", "cmd": cmd, "status": "REJECTED", "msg": "Robot must be homed first"}
            axis = payload.get("axis", "Z").upper()
            delta = float(payload.get("delta", 0.0))
            speed = float(payload.get("speed", 0.5))
            axis_map = {"X": 0, "Y": 1, "Z": 2, "RX": 3, "RY": 4, "RZ": 5, "ROLL": 3, "PITCH": 4, "YAW": 5}
            if axis in axis_map:
                asyncio.create_task(self.jog_cartesian(axis_map[axis], 1 if delta > 0 else -1, step_size_mm=abs(delta), speed_factor=speed))
                return {"type": "ACK", "cmd": "JOG_CARTESIAN", "status": "OK"}
            return {"type": "ACK", "cmd": "JOG_CARTESIAN", "status": "ERROR", "msg": f"Unknown axis {axis}"}

        elif cmd == "SET_GRIPPER":
            angle = payload.get("angle", payload.get("position", None))
            gap_mm = payload.get("mm", None)
            self.set_gripper(mm=gap_mm, angle=angle)
            return {"type": "ACK", "cmd": "SET_GRIPPER", "status": "OK", "angle": self.last_gripper_angle, "mm": self.gripper_mm}

        elif cmd == "GET_TRIMS":
            return {"type": "TRIMS", "homing_trims_deg": self.homing_trims_deg}

        elif cmd == "SET_TRIMS":
            new_trims = payload.get("trims", {})
            save = payload.get("save", True)
            self.update_trims(new_trims, persist=save)
            return {"type": "ACK", "cmd": "SET_TRIMS", "status": "OK", "trims": self.homing_trims_deg}

        return {"type": "ACK", "cmd": cmd, "status": "UNKNOWN_COMMAND"}

    async def ws_handler(self, websocket: websockets.WebSocketServerProtocol):
        """Handle incoming WebSocket control connections (Port 8765)."""
        self.connected_ws_clients.add(websocket)
        logger.info("WebSocket Client connected: %s", websocket.remote_address)
        try:
            async for raw_msg in websocket:
                try:
                    if isinstance(raw_msg, (bytes, bytearray)):
                        if len(raw_msg) >= 31 and raw_msg[0] == 0xAA:
                            if not verify_crc(raw_msg):
                                continue
                            opcode = raw_msg[1]
                            seq = struct.unpack_from("<H", raw_msg, 2)[0]
                            if opcode == 0x05:  # Heartbeat Ping
                                # Return 31-byte binary pong echoing sequence number
                                pong = bytearray(31)
                                pong[0] = 0xAA
                                pong[1] = 0x05
                                struct.pack_into("<H", pong, 2, seq)
                                xor = 0
                                for i in range(30):
                                    xor ^= pong[i]
                                pong[30] = xor
                                await websocket.send(bytes(pong))
                                continue
                            elif opcode == 0x01:  # SERVO_JOINTS streaming
                                j = struct.unpack_from("<6f", raw_msg, 4)
                                gripper = raw_msg[28]
                                self._sync_homed_from_klipper()
                                if abs(float(gripper) - self.last_gripper_angle) >= 1.0:
                                    self.set_gripper(angle=float(gripper))
                                if self.homed and self.state != "ESTOP":
                                    await self.servo_to_joints(list(j), dt=0.02)
                                continue
                            elif opcode == 0x02:  # SERVO_CARTESIAN streaming
                                p = struct.unpack_from("<6f", raw_msg, 4)
                                gripper = raw_msg[28]
                                self._sync_homed_from_klipper()
                                if abs(float(gripper) - self.last_gripper_angle) >= 1.0:
                                    self.set_gripper(angle=float(gripper))
                                if self.homed and self.state != "ESTOP":
                                    await self.servo_to_cartesian(list(p), dt=0.02)
                                continue
                            else:
                                continue
                        else:
                            # Try decoding as UTF-8 JSON
                            try:
                                payload = json.loads(raw_msg.decode("utf-8"))
                            except Exception:
                                continue
                    else:
                        try:
                            payload = json.loads(raw_msg)
                        except Exception:
                            continue

                    resp = await self.process_command(payload)
                    if resp:
                        await websocket.send(json.dumps(resp))
                except Exception as ex:
                    logger.debug("WebSocket client message error: %s", ex)
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            logger.warning("WebSocket handler exception: %s", e)
        finally:
            self.connected_ws_clients.discard(websocket)
            logger.info("WebSocket Client disconnected: %s", websocket.remote_address)

    # =========================================================================
    # HTTP KIOSK & REST API SERVER (PORT 8000)
    # =========================================================================

    async def handle_http_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        """Non-blocking HTTP server hosting Kiosk UI and fallback REST API endpoints."""
        try:
            line = await reader.readline()
            if not line:
                return
            req_line = line.decode("utf-8", errors="ignore").strip()
            parts = req_line.split(" ")
            method = parts[0].upper() if len(parts) > 0 else "GET"
            path = parts[1] if len(parts) > 1 else "/"

            content_length = 0
            while True:
                h_line = await reader.readline()
                if not h_line or h_line in (b"\r\n", b"\n"):
                    break
                h_str = h_line.decode("utf-8", errors="ignore")
                if h_str.lower().startswith("content-length:"):
                    try:
                        content_length = int(h_str.split(":")[1].strip())
                    except ValueError:
                        pass

            body_bytes = b""
            if content_length > 0:
                body_bytes = await reader.readexactly(content_length)

            cors_headers = (
                "Access-Control-Allow-Origin: *\r\n"
                "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
                "Access-Control-Allow-Headers: Content-Type\r\n"
            )

            if method == "OPTIONS":
                resp = f"HTTP/1.1 204 No Content\r\n{cors_headers}Connection: close\r\n\r\n"
                writer.write(resp.encode("utf-8"))
                await writer.drain()
                return

            # REST Endpoint: Telemetry
            if path.startswith("/api/telemetry") and method == "GET":
                data = json.dumps(self.get_telemetry_dict()).encode("utf-8")
                resp = (
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: application/json\r\n"
                    f"{cors_headers}"
                    f"Content-Length: {len(data)}\r\n"
                    "Connection: close\r\n\r\n"
                )
                writer.write(resp.encode("utf-8"))
                writer.write(data)
                await writer.drain()
                return

            # REST Endpoint: Command Dispatch
            if path.startswith("/api/cmd") and method == "POST":
                try:
                    payload = json.loads(body_bytes.decode("utf-8"))
                    res = await self.process_command(payload)
                    data = json.dumps(res).encode("utf-8")
                    status_line = "HTTP/1.1 200 OK\r\n"
                except Exception as ex:
                    data = json.dumps({"status": "ERROR", "msg": str(ex)}).encode("utf-8")
                    status_line = "HTTP/1.1 400 Bad Request\r\n"
                resp = (
                    f"{status_line}"
                    "Content-Type: application/json\r\n"
                    f"{cors_headers}"
                    f"Content-Length: {len(data)}\r\n"
                    "Connection: close\r\n\r\n"
                )
                writer.write(resp.encode("utf-8"))
                writer.write(data)
                await writer.drain()
                return

            # REST Endpoint: Homing Calibration Trims
            if path.startswith("/api/trims"):
                if method == "GET":
                    data = json.dumps({"homing_trims_deg": self.homing_trims_deg}).encode("utf-8")
                else:
                    try:
                        payload = json.loads(body_bytes.decode("utf-8"))
                        trims = payload.get("trims", payload)
                        self.update_trims(trims, persist=True)
                        data = json.dumps({"status": "OK", "homing_trims_deg": self.homing_trims_deg}).encode("utf-8")
                    except Exception as ex:
                        data = json.dumps({"status": "ERROR", "msg": str(ex)}).encode("utf-8")
                resp = (
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: application/json\r\n"
                    f"{cors_headers}"
                    f"Content-Length: {len(data)}\r\n"
                    "Connection: close\r\n\r\n"
                )
                writer.write(resp.encode("utf-8"))
                writer.write(data)
                await writer.drain()
                return

            # Default: Serve ui/touchscreen.html
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
                if method != "HEAD":
                    writer.write(content)
            else:
                body = b"<h1>PAROL6 UI Not Found</h1>"
                headers = (
                    "HTTP/1.1 404 Not Found\r\n"
                    "Content-Type: text/html; charset=UTF-8\r\n"
                    f"Content-Length: {len(body)}\r\n"
                    "Connection: close\r\n\r\n"
                )
                writer.write(headers.encode("utf-8"))
                if method != "HEAD":
                    writer.write(body)

            await writer.drain()
        except Exception:
            pass
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
        logger.info("Starting PAROL6 Engine Servers (6-Axis Klipper Stack)...")

        # 1. Start WebSocket Server (Port 8765)
        ws_server = await websockets.serve(self.ws_handler, "0.0.0.0", WS_PORT)
        logger.info("WebSocket Control Server active on ws://0.0.0.0:%d", WS_PORT)

        # 2. Start Raw UDP Motion Server (Port 8766)
        loop = asyncio.get_running_loop()
        udp_transport, _ = await loop.create_datagram_endpoint(
            lambda: UdpMotionProtocol(self),
            local_addr=("0.0.0.0", UDP_PORT)
        )
        self.udp_transport = udp_transport
        logger.info("Raw UDP Motion Streamer active on 0.0.0.0:%d", UDP_PORT)

        # 3. Start HTTP Kiosk Server (Port 8000)
        http_server = await asyncio.start_server(self.handle_http_client, "0.0.0.0", HTTP_PORT)
        logger.info("Touchscreen HTTP Kiosk Server active on http://0.0.0.0:%d", HTTP_PORT)

        # 4. Start Klipper Client Bridge ONLY after network ports are exclusively bound
        self.klipper.start()
        self._sync_homed_from_klipper()

        # 5. Launch background tasks
        tasks = [
            asyncio.create_task(self.broadcast_telemetry()),
            asyncio.create_task(self.jitter_buffer_worker()),
            asyncio.create_task(self.udp_telemetry_loop()),
        ]

        logger.info("PAROL6 Klipper Engine operational. Ready for commands.")
        try:
            await asyncio.gather(*tasks)
        except asyncio.CancelledError:
            pass
        finally:
            self.klipper.stop()
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

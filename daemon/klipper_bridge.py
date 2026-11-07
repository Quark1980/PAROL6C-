#!/usr/bin/env python3
"""
Klipper Motion Bridge for PAROL6 6-DOF Robotic Arm
===================================================
High-performance asynchronous client bridge connecting the PAROL6 Python Daemon
to the 6-Axis Klipper core via Unix domain pseudo-terminal (/tmp/printer).

Provides:
- Non-blocking G-code dispatch and stream rate-limiting.
- Multi-axis coordinated move streaming (G1 X... Y... Z... A... B... C... F...).
- Hardware E-Stop via M112 and MCU reset via FIRMWARE_RESTART.
- Periodic background position queries (M114) and status synchronization.
- Auto-reconnect handling if Klipper service restarts.
"""

from __future__ import annotations

import errno
import json
import os
import re
import select
import socket
import sys
import threading
import time
import logging
from typing import List, Optional, Tuple

try:
    import tty
    import termios
except ImportError:
    tty = None
    termios = None

logger = logging.getLogger("klipper_bridge")

M114_REGEX = re.compile(
    r"X:([-\d\.]+)\s+Y:([-\d\.]+)\s+Z:([-\d\.]+)\s+A:([-\d\.]+)\s+B:([-\d\.]+)\s+C:([-\d\.]+)"
)

class KlipperBridge:
    def __init__(
        self,
        pty_path: str = "/tmp/printer",
        fallback_path: str = "/home/mark/printer_data/comms/klippy.serial",
        uds_path: str = "/home/mark/printer_data/comms/klippy.sock",
        query_rate_hz: float = 10.0,
    ):
        self.pty_path = pty_path
        self.fallback_path = fallback_path
        self.uds_path = uds_path
        self.query_interval_s = 1.0 / max(1.0, query_rate_hz)

        self._fd: Optional[int] = None
        self._lock = threading.Lock()
        self._running = False
        self._worker_thread: Optional[threading.Thread] = None

        self.is_connected = False
        self.is_ready = False
        self.is_homed = False
        self.is_homing = False
        self.homing_succeeded = False
        self.homed_mask = 0x00
        self._homing_done_event: Optional[threading.Event] = None
        self.last_error: str = ""
        self.on_comm_log = None
        self._stream_tx_count: int = 0

        self.actual_joints_deg: List[float] = [0.0, -90.0, 180.0, 0.0, 0.0, 180.0]
        self.last_response_time = 0.0
        self.last_query_time = 0.0
        self.last_uds_sync_time = 0.0
        self._last_sent_servo_angle: float = -1.0

    def sync_klipper_status_uds(self) -> bool:
        """Query Klipper Unix Domain Socket for toolhead homed_axes and live position."""
        if not os.path.exists(self.uds_path):
            return False
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(0.5)
            sock.connect(self.uds_path)
            req = {
                "id": 9001,
                "method": "objects/query",
                "params": {"objects": {"toolhead": ["homed_axes", "position"]}},
            }
            sock.sendall(json.dumps(req).encode("utf-8") + b"\x03")
            buf = b""
            while b"\x03" not in buf:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                buf += chunk
            sock.close()
            if b"\x03" in buf:
                msg = json.loads(buf.split(b"\x03", 1)[0].decode("utf-8"))
                th = msg.get("result", {}).get("status", {}).get("toolhead", {})
                homed_axes = str(th.get("homed_axes", "")).lower()
                pos = th.get("position", [])
                if len(pos) >= 6:
                    self.actual_joints_deg = [float(pos[i]) for i in range(6)]
                    self.is_ready = True
                mask = 0
                for idx, ax in enumerate(["x", "y", "z", "a", "b", "c"]):
                    if ax in homed_axes:
                        mask |= (1 << idx)
                self.homed_mask = mask
                self.is_homed = (mask == 0x3F)
                return True
        except Exception as e:
            logger.debug("UDS status sync error: %s", e)
        return False

    def connect(self) -> bool:
        """Establish non-blocking raw connection to Klipper pseudo-terminal."""
        with self._lock:
            if self._fd is not None:
                try:
                    os.close(self._fd)
                except Exception:
                    pass
                self._fd = None

            path_to_open = self.pty_path
            if not os.path.exists(path_to_open) and os.path.exists(self.fallback_path):
                path_to_open = self.fallback_path

            if not os.path.exists(path_to_open):
                logger.warning("Klipper PTY not found at %s or %s", self.pty_path, self.fallback_path)
                self.is_connected = False
                return False

            try:
                self._fd = os.open(path_to_open, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                if tty is not None and termios is not None:
                    try:
                        tty.setraw(self._fd, termios.TCSANOW)
                    except Exception as te:
                        logger.debug("termios setraw note: %s", te)

                self.is_connected = True
                logger.info("Connected to Klipper PTY at %s (fd=%d)", path_to_open, self._fd)

                # Flush any pending buffer
                try:
                    os.read(self._fd, 8192)
                except BlockingIOError:
                    pass

                # Synchronize initial homed state and position via Klipper UDS
                self.sync_klipper_status_uds()
                return True
            except Exception as e:
                logger.error("Failed to open Klipper PTY %s: %s", path_to_open, e)
                self.is_connected = False
                self._fd = None
                return False

    def start(self):
        """Start background polling worker thread."""
        if self._running:
            return
        self._running = True
        self.connect()
        self._worker_thread = threading.Thread(target=self._worker_loop, daemon=True, name="klipper_bridge_worker")
        self._worker_thread.start()
        logger.info("Klipper bridge background worker started.")

    def stop(self):
        """Stop background worker and close file descriptor."""
        self._running = False
        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.0)
        with self._lock:
            if self._fd is not None:
                try:
                    os.close(self._fd)
                except Exception:
                    pass
                self._fd = None
            self.is_connected = False
            self.is_ready = False

    def _worker_loop(self):
        """Background thread servicing PTY reads and dispatching periodic M114 queries."""
        read_buf = b""
        while self._running:
            now = time.monotonic()

            # Auto-reconnect if connection lost
            if not self.is_connected or self._fd is None:
                time.sleep(1.0)
                self.connect()
                continue

            # Poll for incoming data from Klipper
            try:
                rlist, _, _ = select.select([self._fd], [], [], 0.02)
                if rlist:
                    chunk = os.read(self._fd, 4096)
                    if not chunk:
                        logger.warning("Klipper PTY EOF encountered. Reconnecting...")
                        self.is_connected = False
                        continue
                    read_buf += chunk
                    while b"\n" in read_buf:
                        line, read_buf = read_buf.split(b"\n", 1)
                        line_str = line.decode("utf-8", errors="ignore").strip()
                        if line_str:
                            self._parse_line(line_str)
            except (BlockingIOError, select.error):
                pass
            except Exception as e:
                logger.error("Error reading Klipper PTY: %s", e)
                self.is_connected = False
                continue

            # Periodic M114 query for actual position tracking (10 Hz, paused during homing)
            if self.is_connected and not self.is_homing and (now - self.last_query_time >= self.query_interval_s):
                self.last_query_time = now
                self.send_gcode("M114", log_cmd=False)

            # Periodic UDS homed_axes synchronization (every 2.0s if not homed)
            if self.is_connected and not self.is_homing and not self.is_homed and (now - self.last_uds_sync_time >= 2.0):
                self.last_uds_sync_time = now
                self.sync_klipper_status_uds()

    def _emit_log(self, direction: str, msg: str):
        if self.on_comm_log is not None:
            try:
                self.on_comm_log("RPI_MCU", direction, msg)
            except Exception:
                pass

    def _parse_line(self, line: str):
        """Parse incoming line from Klipper."""
        self.last_response_time = time.monotonic()

        # Position report M114
        m = M114_REGEX.search(line)
        if m:
            try:
                coords = [float(m.group(i)) for i in range(1, 7)]
                self.actual_joints_deg = coords
                self.is_ready = True
                return
            except ValueError:
                pass

        if line == "ok":
            return

        # Readiness and state events
        if "Klipper state: Ready" in line:
            self.is_ready = True
            self.last_error = ""
            logger.info("Klipper reports READY state.")
            self._emit_log("RX", line)
        elif "Klipper state: Shutdown" in line or "Printer is shutdown" in line:
            self.is_ready = False
            self.homing_succeeded = False
            self.last_error = line
            logger.warning("Klipper reports SHUTDOWN state.")
            self._emit_log("ERR", line)
            if self.is_homing and self._homing_done_event:
                self._homing_done_event.set()
        elif "Klipper state: Disconnect" in line:
            self.is_ready = False
            self.is_connected = False
            self.homing_succeeded = False
            self.last_error = line
            logger.warning("Klipper reports DISCONNECTED state.")
            self._emit_log("ERR", line)
            if self.is_homing and self._homing_done_event:
                self._homing_done_event.set()
        elif line.startswith("!!"):
            self.homing_succeeded = False
            self.last_error = line
            if "is not valid in section" in line or "Config error" in line or "Must home" in line:
                if "is not valid in section" in line or "Config error" in line:
                    self.is_ready = False
            logger.error("Klipper error: %s", line)
            self._emit_log("ERR", line)
            if self.is_homing and self._homing_done_event:
                self._homing_done_event.set()
        elif "PAROL6 calibrated to CAD home" in line or "CAD positie ingesteld" in line:
            self.homing_succeeded = True
            logger.info("PAROL6 homing sequence confirmed by Klipper: %s", line)
            self._emit_log("RX", line)
            if self.is_homing and self._homing_done_event:
                self._homing_done_event.set()
        else:
            self._emit_log("RX", line)

    def send_gcode(self, cmd: str, log_cmd: bool = True) -> bool:
        """Send a single G-code line to Klipper with EAGAIN retry protection."""
        clean_cmd = cmd.strip()
        if not clean_cmd:
            return False

        with self._lock:
            if not self.is_connected or self._fd is None:
                if log_cmd:
                    self._emit_log("ERR", f"TX DROPPED (Disconnected): {clean_cmd}")
                return False
            packet = (clean_cmd + "\n").encode("utf-8")
            for attempt in range(3):
                try:
                    os.write(self._fd, packet)
                    if log_cmd:
                        logger.debug("Sent G-code: %s", clean_cmd)
                        self._emit_log("TX", clean_cmd)
                    return True
                except BlockingIOError:
                    # PTY buffer temporarily full; wait up to 20ms for Klipper to drain
                    try:
                        select.select([], [self._fd], [], 0.02)
                    except Exception:
                        pass
                except OSError as e:
                    if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                        try:
                            select.select([], [self._fd], [], 0.02)
                        except Exception:
                            pass
                        continue
                    logger.error("Failed to write G-code '%s': %s", clean_cmd, e)
                    self._emit_log("ERR", f"Write error '{clean_cmd}': {e}")
                    self.is_connected = False
                    return False
                except Exception as e:
                    logger.error("Failed to write G-code '%s': %s", clean_cmd, e)
                    self._emit_log("ERR", f"Write error '{clean_cmd}': {e}")
                    self.is_connected = False
                    return False
            if log_cmd:
                logger.warning("Dropped G-code '%s' after 3 EAGAIN retries", clean_cmd)
                self._emit_log("ERR", f"Dropped G-code after 3 retries: {clean_cmd}")
            return False

    def stream_target(
        self,
        j1: float,
        j2: float,
        j3: float,
        j4: float,
        j5: float,
        j6: float,
        feedrate_deg_min: float = 3000.0,
    ) -> bool:
        """Stream a 6-axis synchronous coordinated move command (G1)."""
        cmd = (
            f"G1 X{j1:.3f} Y{j2:.3f} Z{j3:.3f} A{j4:.3f} B{j5:.3f} C{j6:.3f} "
            f"F{feedrate_deg_min:.1f}"
        )
        ok = self.send_gcode(cmd, log_cmd=False)
        self._stream_tx_count += 1
        if self._stream_tx_count % 15 == 1:
            self._emit_log("TX", f"[STREAM #{self._stream_tx_count}] {cmd}")
        return ok

    def emergency_stop(self):
        """Trigger instant hardware shutdown via Klipper M112."""
        logger.critical("EMERGENCY STOP (M112) DISPATCHED TO KLIPPER!")
        self.send_gcode("M112", log_cmd=True)
        self.is_ready = False

    def firmware_restart(self):
        """Reset MCU and reinitialize Klipper firmware."""
        logger.info("Dispatching FIRMWARE_RESTART to Klipper...")
        self.send_gcode("FIRMWARE_RESTART", log_cmd=True)
        self.is_ready = False

    def set_kinematic_position(
        self,
        j1: float,
        j2: float,
        j3: float,
        j4: float,
        j5: float,
        j6: float,
    ):
        """Force internal coordinate registers to given joint values and mark all homed."""
        cmd = f"SET_KINEMATIC_POSITION X={j1:.3f} Y={j2:.3f} Z={j3:.3f} A={j4:.3f} B={j5:.3f} C={j6:.3f}"
        self.send_gcode(cmd, log_cmd=True)
        self.actual_joints_deg = [j1, j2, j3, j4, j5, j6]
        self.homed_mask = 0x3F
        self.is_homed = True

    def set_cad_home(self):
        """Invoke SET_CAD_HOME macro in Klipper."""
        logger.info("Invoking SET_CAD_HOME macro in Klipper...")
        self.send_gcode("SET_CAD_HOME", log_cmd=True)
        self.actual_joints_deg = [0.0, -90.0, 180.0, 0.0, 0.0, 180.0]
        self.homed_mask = 0x3F
        self.is_homed = True

    def home_all(self, timeout_s: float = 120.0) -> bool:
        """Execute full PAROL6 sequential homing sequence and wait for completion."""
        logger.info("Executing HOME sequence via Klipper (timeout=%.1fs)...", timeout_s)
        self.is_homing = True
        self.homing_succeeded = False
        self._homing_done_event = threading.Event()
        self.send_gcode("HOME", log_cmd=True)

        finished = self._homing_done_event.wait(timeout=timeout_s)
        self.is_homing = False

        if not finished or not self.is_ready or not self.homing_succeeded:
            logger.warning("Homing sequence timed out or failed! finished=%s, ready=%s, success=%s",
                           finished, self.is_ready, self.homing_succeeded)
            return False

        # Query live coordinates once after homing
        time.sleep(0.15)
        self.send_gcode("M114", log_cmd=True)
        self.homed_mask = 0x3F
        self.is_homed = True
        logger.info("Klipper reports homing sequence complete.")
        return True

    def home_joint(self, joint_idx: int):
        """Home a specific joint (0-5 -> J1-J6)."""
        axis_names = ["X", "Y", "Z", "A", "B", "C"]
        if 0 <= joint_idx < 6:
            axis = axis_names[joint_idx]
            self.send_gcode(f"G28 {axis}", log_cmd=True)
            self.homed_mask |= (1 << joint_idx)

    def set_gripper(self, mm: Optional[float] = None, angle: Optional[float] = None):
        """Actuate gripper servo directly via Klipper SET_SERVO on RAMPS D11 (ar11 / PB5)."""
        if angle is not None:
            ang = max(0.0, min(180.0, float(angle)))
        elif mm is not None:
            gap = max(0.0, min(62.5, float(mm)))
            ang = 180.0 * (1.0 - (gap / 62.5))
        else:
            return
        if abs(ang - self._last_sent_servo_angle) < 0.25 and ang not in (0.0, 90.0, 180.0):
            return
        self._last_sent_servo_angle = ang
        self.send_gcode(f"SET_SERVO SERVO=gripper ANGLE={ang:.1f}", log_cmd=True)

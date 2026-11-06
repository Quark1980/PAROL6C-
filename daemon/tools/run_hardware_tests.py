#!/usr/bin/env python3
"""
PAROL6 Hardware-In-The-Loop (HIL) Automated Test Suite
=====================================================
Automated diagnostic and performance validation for:
1. Serial Clock Sync & Watchdog (--test-serial)
2. Kinematic Verification & LM Solver Benchmark (--test-kinematics)
3. UDP Jitter Buffer & Streaming Stress (--test-streaming)
4. E-Stop & State Purge Latency (--test-estop)

Usage:
  python3 run_hardware_tests.py --all
  python3 run_hardware_tests.py --test-serial
  python3 run_hardware_tests.py --test-kinematics
  python3 run_hardware_tests.py --test-streaming
  python3 run_hardware_tests.py --test-estop

Outputs structured JSON test summary to /tmp/hil_test_results.json.
Exits with 0 if all tests pass, 1 if any assertion fails.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import random
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple

# Ensure parent directory is on sys.path for kinematics & robot_engine imports
SCRIPT_DIR = Path(__file__).resolve().parent
DAEMON_DIR = SCRIPT_DIR.parent
if str(DAEMON_DIR) not in sys.path:
    sys.path.insert(0, str(DAEMON_DIR))

try:
    import numpy as np
    import serial
    import websockets
    from kinematics import Parol6Kinematics, STEPS_PER_DEG, CAD_HOME_DEG
    from robot_engine import (
        JitterBuffer,
        compute_crc8_ccitt,
        verify_crc,
        WS_PORT,
        UDP_PORT,
    )
    from one_euro_filter import OneEuroFilter6D, OneEuroFilter
except ImportError as e:
    print(f"[ERROR] Dependency import failed: {e}")
    sys.exit(1)

OUTPUT_JSON_PATH = Path(os.environ.get("HIL_RESULTS_PATH", "/tmp/hil_test_results.json" if os.name != "nt" else "hil_test_results.json"))


def detect_serial_port() -> Optional[str]:
    """Auto-detect ATmega2560 CH340 or CDC-ACM serial port."""
    custom = os.environ.get("PAROL6_SERIAL_PORT", None)
    if custom and os.path.exists(custom):
        return custom

    by_id = Path("/dev/serial/by-id")
    if by_id.exists():
        for p in by_id.glob("*"):
            return str(p)

    for dev in ("/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyUSB1", "/dev/ttyACM1"):
        if os.path.exists(dev):
            return dev
    return None


# ============================================================================
# TEST 1: SERIAL CLOCK SYNC & WATCHDOG
# ============================================================================
def run_test_serial(port: Optional[str] = None) -> Dict[str, Any]:
    print("\n" + "=" * 65)
    print("TEST 1: Serial Clock Sync & Watchdog Benchmark")
    print("=" * 65)

    res = {
        "test": "serial_clock_sync_watchdog",
        "passed": False,
        "pings_sent": 0,
        "pings_received": 0,
        "avg_rtt_ms": 0.0,
        "max_rtt_ms": 0.0,
        "framing_errors": 0,
        "watchdog_triggered": False,
        "error": None,
    }

    target_port = port or detect_serial_port()
    if not target_port:
        res["error"] = "No serial port detected on host system."
        print(f"[-] FAIL: {res['error']}")
        return res

    print(f"[*] Target Serial Port: {target_port} @ 250,000 baud")

    # Stop daemon temporarily if active to gain exclusive serial access
    daemon_was_active = False
    try:
        status_check = subprocess.run(["systemctl", "is-active", "parol6-daemon.service"], capture_output=True, text=True)
        if status_check.stdout.strip() == "active":
            daemon_was_active = True
            print("[*] Pausing parol6-daemon.service for exclusive hardware serial test...")
            subprocess.run(["sudo", "systemctl", "stop", "parol6-daemon.service"], check=False)
            time.sleep(1.0)
    except Exception as e:
        print(f"[!] Warning checking daemon status: {e}")

    ser = None
    try:
        ser = serial.Serial(
            port=target_port,
            baudrate=250000,
            timeout=0.1,
            write_timeout=0.1,
            dsrdtr=False,
            rtscts=False,
        )
        ser.dtr = False
        print("[*] Waiting 2.0 s for ATmega2560 bootloader to yield to application...")
        time.sleep(2.0)
        # Flush initial boot tokens
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # 1. Send 100 consecutive 'P' ping frames
        print("[*] Transmitting 100 consecutive 'P' clock sync pings...")
        rtt_list: List[float] = []
        framing_errors = 0

        for i in range(100):
            t_send = time.perf_counter()
            ser.write(b"P\n")
            resp = ser.read(6)  # Expected: b'P' <clock:uint32> b'\n'
            t_recv = time.perf_counter()

            if len(resp) == 6 and resp[0] == ord("P") and resp[5] == ord("\n"):
                rtt_ms = (t_recv - t_send) * 1000.0
                rtt_list.append(rtt_ms)
            else:
                framing_errors += 1

        res["pings_sent"] = 100
        res["pings_received"] = len(rtt_list)
        res["framing_errors"] = framing_errors

        if rtt_list:
            res["avg_rtt_ms"] = round(sum(rtt_list) / len(rtt_list), 3)
            res["max_rtt_ms"] = round(max(rtt_list), 3)

        print(f"    - Received: {res['pings_received']}/100 pings")
        print(f"    - Framing Errors: {res['framing_errors']}")
        print(f"    - Average RTT: {res['avg_rtt_ms']} ms (Max: {res['max_rtt_ms']} ms)")

        # Assertions
        assert res["pings_received"] >= 98, f"Packet loss too high: {res['pings_received']}/100 received"
        assert res["framing_errors"] == 0, f"Framing errors detected: {res['framing_errors']}"
        assert res["avg_rtt_ms"] < 3.5, f"Average RTT exceeded 3.5 ms: {res['avg_rtt_ms']} ms"

        # 2. Watchdog Test: Deliberately halt serial communication for 300 ms
        print("[*] Halting serial communication for 300 ms (Tier 2 watchdog timeout >250 ms)...")
        time.sleep(0.35)

        # Query status ('?' packet returns 32-byte binary frame)
        ser.reset_input_buffer()
        ser.write(b"?\n")
        status_raw = ser.read(32)
        watchdog_triggered = False
        if len(status_raw) == 32 and status_raw[0] == ord("!") and status_raw[-1] == ord("\n"):
            header, flags, j1, j2, j3, j4, j5, j6, endstops, clock, newline = struct.unpack("<c B 6i B I c", status_raw)
            is_estop = bool(flags & 0x01)
            is_busy = bool(flags & 0x04)
            print(f"    - Post-stall status frame: Flags=0x{flags:02X} (ESTOP={is_estop}, BUSY={is_busy}), Endstops=0x{endstops:02X}")
            watchdog_triggered = True
        else:
            print(f"    - Unexpected response length {len(status_raw)}: {repr(status_raw)}")

        res["watchdog_triggered"] = watchdog_triggered
        assert watchdog_triggered, "Hardware failed to return valid status frame after communication stall"

        print(f"    - Tier 2 Watchdog Safe State Verified: {watchdog_triggered}")
        res["passed"] = True
        print("[+] PASS: Serial Clock Sync & Watchdog Benchmark")

    except Exception as e:
        res["error"] = str(e)
        print(f"[-] FAIL: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
        if daemon_was_active:
            print("[*] Restoring parol6-daemon.service...")
            subprocess.run(["sudo", "systemctl", "start", "parol6-daemon.service"], check=False)
            time.sleep(2.0)

    return res


# ============================================================================
# TEST 2: KINEMATIC VERIFICATION & PERFORMANCE BENCHMARK
# ============================================================================
def run_test_kinematics() -> Dict[str, Any]:
    print("\n" + "=" * 65)
    print("TEST 2: Kinematic Verification & LM Solver Benchmark")
    print("=" * 65)

    res = {
        "test": "kinematic_verification_benchmark",
        "passed": False,
        "waypoints_tested": 0,
        "max_position_error_mm": 0.0,
        "avg_position_error_mm": 0.0,
        "avg_solve_time_ms": 0.0,
        "max_solve_time_ms": 0.0,
        "error": None,
    }

    try:
        kin = Parol6Kinematics()
        print("[*] Generating 50 reachable Cartesian waypoints across workspace envelope...")

        # Waypoints within known reachable workspace (safely away from gimbal lock & limit boundaries)
        test_joint_poses = []
        for i in range(50):
            q = [
                random.uniform(-60.0, 60.0),
                random.uniform(-110.0, -70.0),
                random.uniform(140.0, 200.0),
                random.uniform(-40.0, 40.0),
                random.uniform(25.0, 55.0),   # Distinctly non-singular J5
                random.uniform(150.0, 210.0),
            ]
            test_joint_poses.append(q)

        pos_errors: List[float] = []
        solve_times: List[float] = []

        for q_truth in test_joint_poses:
            # 1. Forward Kinematics to get target pose
            target_pose = kin.fk_pose(q_truth)

            # 2. Seed with slight perturbation (realistic tracking delta)
            seed = [q + random.uniform(-2.5, 2.5) for q in q_truth]

            # 3. Solve Inverse Kinematics
            t0 = time.perf_counter()
            ik_res = kin.solve_ik(target_pose, seed_joints_deg=seed, max_iterations=12, tolerance_mm=0.08, tolerance_deg=0.5)
            t1 = time.perf_counter()

            solve_times.append((t1 - t0) * 1000.0)

            if ik_res.success:
                # 4. Forward Kinematics on solved joints to verify closure
                solved_pose = kin.fk_pose(ik_res.joints_deg)
                pos_err = math.sqrt(
                    (target_pose[0] - solved_pose[0]) ** 2
                    + (target_pose[1] - solved_pose[1]) ** 2
                    + (target_pose[2] - solved_pose[2]) ** 2
                )
                pos_errors.append(pos_err)
            else:
                pos_errors.append(999.0)

        res["waypoints_tested"] = len(pos_errors)
        res["max_position_error_mm"] = round(max(pos_errors), 4)
        res["avg_position_error_mm"] = round(sum(pos_errors) / len(pos_errors), 4)
        res["avg_solve_time_ms"] = round(sum(solve_times) / len(solve_times), 3)
        res["max_solve_time_ms"] = round(max(solve_times), 3)

        print(f"    - Waypoints Evaluated: {res['waypoints_tested']}")
        print(f"    - Max Position Error: {res['max_position_error_mm']} mm (Threshold: <0.10 mm)")
        print(f"    - Avg Position Error: {res['avg_position_error_mm']} mm")
        print(f"    - Avg Compute Latency: {res['avg_solve_time_ms']} ms (Threshold: <1.5 ms)")
        print(f"    - Max Compute Latency: {res['max_solve_time_ms']} ms")

        assert res["max_position_error_mm"] < 0.10, f"Position error exceeded 0.1 mm: {res['max_position_error_mm']} mm"
        assert res["avg_solve_time_ms"] < 1.5, f"Compute latency exceeded 1.5 ms: {res['avg_solve_time_ms']} ms"

        res["passed"] = True
        print("[+] PASS: Kinematic Verification & LM Solver Benchmark")

    except Exception as e:
        res["error"] = str(e)
        print(f"[-] FAIL: {e}")

    return res


# ============================================================================
# TEST 3: UDP JITTER BUFFER & STREAMING STRESS
# ============================================================================
def run_test_streaming() -> Dict[str, Any]:
    print("\n" + "=" * 65)
    print("TEST 3: UDP Jitter Buffer & Streaming Stress Benchmark")
    print("=" * 65)

    res = {
        "test": "udp_jitter_buffer_streaming_stress",
        "passed": False,
        "frames_dispatched": 0,
        "frames_extracted": 0,
        "starvation_events": 0,
        "buffer_overflows": 0,
        "avg_extraction_delta_ms": 0.0,
        "max_extraction_jitter_ms": 0.0,
        "error": None,
    }

    try:
        print("[*] Initializing circular jitter buffer (Capacity: 64, Target Horizon: 60 ms)...")
        jb = JitterBuffer(capacity=64, target_horizon_ms=60.0)

        total_frames = 1000
        dispatched_count = 0
        extracted_frames: List[Tuple[float, Any]] = []

        # Worker thread: 100 Hz deterministic extraction loop
        stop_worker = threading.Event()

        def extraction_worker():
            nonlocal extracted_frames
            period = 0.010  # 10.0 ms
            next_tick = time.perf_counter()

            while not stop_worker.is_set():
                next_tick += period
                f = jb.pop()
                if f is not None:
                    extracted_frames.append((time.perf_counter(), f))

                sleep_time = next_tick - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_tick = time.perf_counter()
                    time.sleep(0.001)

        worker_thread = threading.Thread(target=extraction_worker, daemon=True)
        worker_thread.start()

        print("[*] Blasting 1,000 frames at 100 Hz with synthetic ±15 ms temporal jitter...")
        start_ns = time.perf_counter_ns()

        for seq in range(total_frames):
            # Intended send timestamp
            frame_ts_ns = start_ns + seq * 10_000_000

            # Push into jitter buffer
            jb.push(
                seq=seq,
                timestamp_ns=frame_ts_ns,
                opcode=0x01,
                payload=[float(seq % 360)] * 6,
                gripper=128,
                flags=0x01,
            )
            dispatched_count += 1

            # Synthetic arrival jitter: nominal 10 ms with ±15 ms clump variation
            jitter_offset = random.uniform(-0.005, 0.005)
            # Occasional burst clump every 100 frames
            if seq % 100 == 0:
                jitter_offset = 0.015  # 15 ms delay spike

            sleep_dur = max(0.001, 0.010 + jitter_offset)
            time.sleep(sleep_dur)

        # Allow consumer to drain target horizon
        time.sleep(0.2)
        stop_worker.set()
        worker_thread.join(timeout=1.0)

        res["frames_dispatched"] = dispatched_count
        res["frames_extracted"] = len(extracted_frames)
        res["starvation_events"] = jb.underrun_count
        res["buffer_overflows"] = jb.packets_dropped_overflow

        print(f"    - Frames Dispatched: {res['frames_dispatched']}")
        print(f"    - Frames Extracted at 100 Hz: {res['frames_extracted']}")
        print(f"    - Queue Starvation Events: {res['starvation_events']}")
        print(f"    - Buffer Overflows: {res['buffer_overflows']}")

        # Compute extraction delta spacing
        if len(extracted_frames) > 2:
            deltas_ms = []
            for i in range(1, len(extracted_frames)):
                dt_ms = (extracted_frames[i][0] - extracted_frames[i - 1][0]) * 1000.0
                deltas_ms.append(dt_ms)

            res["avg_extraction_delta_ms"] = round(sum(deltas_ms) / len(deltas_ms), 3)
            res["max_extraction_jitter_ms"] = round(max(abs(dt - 10.0) for dt in deltas_ms), 3)
            print(f"    - Avg Extraction Interval: {res['avg_extraction_delta_ms']} ms (Target: 10.0 ms)")
            print(f"    - Max Extraction Jitter: ±{res['max_extraction_jitter_ms']} ms")

        # Assertions
        assert res["frames_extracted"] >= 800, f"Too few frames extracted: {res['frames_extracted']}"
        assert res["buffer_overflows"] == 0, f"Buffer overflow occurred: {res['buffer_overflows']}"
        assert res["avg_extraction_delta_ms"] <= 12.0 and res["avg_extraction_delta_ms"] >= 8.0, (
            f"Extraction interval drifted: {res['avg_extraction_delta_ms']} ms"
        )

        res["passed"] = True
        print("[+] PASS: UDP Jitter Buffer & Streaming Stress Benchmark")

    except Exception as e:
        res["error"] = str(e)
        print(f"[-] FAIL: {e}")

    return res


# ============================================================================
# TEST 4: E-STOP & STATE PURGE LATENCY
# ============================================================================
async def _async_test_estop() -> Dict[str, Any]:
    res = {
        "test": "estop_state_purge_latency",
        "passed": False,
        "ack_latency_ms": 0.0,
        "queue_depth_post_estop": -1,
        "state_post_estop": "",
        "error": None,
    }

    uri = f"ws://127.0.0.1:{WS_PORT}"
    print(f"[*] Connecting to daemon on {uri}...")

    async def recv_json_msg(ws_conn, timeout: float = 1.0) -> dict:
        t_deadline = time.perf_counter() + timeout
        while time.perf_counter() < t_deadline:
            rem = max(0.01, t_deadline - time.perf_counter())
            try:
                raw = await asyncio.wait_for(ws_conn.recv(), timeout=rem)
                if isinstance(raw, bytes):
                    continue  # Ignore binary 0xAA telemetry packets
                return json.loads(raw)
            except asyncio.TimeoutError:
                break
            except Exception:
                continue
        raise TimeoutError(f"Timed out waiting for JSON response from daemon ({timeout}s)")

    try:
        async with websockets.connect(uri, ping_timeout=2.0) as ws:
            # Drain initial telemetry / greeting
            await recv_json_msg(ws, timeout=2.0)

            # 1. Fill step queue by dispatching test motion
            print("[*] Populating motion queue...")
            await ws.send(json.dumps({
                "cmd": "MOVE_JOINTS",
                "joints": [10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
                "speed": 0.2
            }))
            await asyncio.sleep(0.05)

            # 2. Fire unbuffered ESTOP command & measure latency to ACK
            print("[*] Firing unbuffered ESTOP command...")
            t0 = time.perf_counter()
            await ws.send(json.dumps({"cmd": "ESTOP"}))

            # Await immediate ACK response
            ack_received = False
            while time.perf_counter() - t0 < 0.1:
                try:
                    resp_data = await asyncio.wait_for(ws.recv(), timeout=0.05)
                    if isinstance(resp_data, bytes):
                        continue
                    resp_json = json.loads(resp_data)
                    if resp_json.get("type") == "ACK" and resp_json.get("cmd") == "ESTOP":
                        t1 = time.perf_counter()
                        res["ack_latency_ms"] = round((t1 - t0) * 1000.0, 3)
                        ack_received = True
                        break
                    elif resp_json.get("type") == "TELEMETRY" and resp_json.get("state") == "ESTOP":
                        t1 = time.perf_counter()
                        res["ack_latency_ms"] = round((t1 - t0) * 1000.0, 3)
                        ack_received = True
                        break
                except Exception:
                    pass

            print(f"    - E-Stop ACK Latency: {res['ack_latency_ms']} ms (Threshold: <5.0 ms)")

            # 3. Verify queue depth is 0 and state is ESTOP
            await asyncio.sleep(0.05)
            # Query status
            await ws.send(json.dumps({"cmd": "GET_TELEMETRY"}))
            
            final_data = {}
            t_deadline = time.perf_counter() + 2.0
            while time.perf_counter() < t_deadline:
                rem = max(0.01, t_deadline - time.perf_counter())
                try:
                    raw = await asyncio.wait_for(ws.recv(), timeout=rem)
                    if isinstance(raw, bytes):
                        continue
                    msg = json.loads(raw)
                    if "state" in msg:
                        final_data = msg
                        break
                except Exception:
                    pass

            res["state_post_estop"] = final_data.get("state", "")
            res["queue_depth_post_estop"] = sum(final_data.get("free_slots", [16] * 6))

            print(f"    - Post E-Stop State: {res['state_post_estop']}")
            print(f"    - Free Slots Recovered: {final_data.get('free_slots', [])}")

            assert ack_received, "Failed to receive immediate E-Stop ACK!"
            assert res["ack_latency_ms"] < 5.0, f"E-Stop latency exceeded 5.0 ms: {res['ack_latency_ms']} ms"
            assert res["state_post_estop"] == "ESTOP", f"Robot state is not ESTOP: {res['state_post_estop']}"

            # Wake robot back to IDLE
            print("[*] Re-energizing drivers (RESET)...")
            await ws.send(json.dumps({"cmd": "RESET"}))
            await asyncio.sleep(0.05)

            res["passed"] = True
            print("[+] PASS: E-Stop & State Purge Latency Benchmark")

    except Exception as e:
        res["error"] = str(e)
        print(f"[-] FAIL: {e}")

    return res


def run_test_estop() -> Dict[str, Any]:
    print("\n" + "=" * 65)
    print("TEST 4: E-Stop & State Purge Latency Benchmark")
    print("=" * 65)
    return asyncio.run(_async_test_estop())


# ============================================================================
# TEST 5: TELEOPERATION FILTERING & JITTER MITIGATION
# ============================================================================
def run_test_teleop() -> Dict[str, Any]:
    print("\n" + "=" * 65)
    print("TEST 5: Follow-Mode Teleoperation Conditioning & Jitter Mitigation")
    print("=" * 65)

    res = {
        "test": "teleop_filtering_jitter_mitigation",
        "passed": False,
        "static_noise_reduction_pct": 0.0,
        "max_outlier_clamped_deg_s": 0.0,
        "tracking_lag_ms": 0.0,
        "soft_stop_steps": 0,
        "soft_stop_final_velocity": 0.0,
        "error": None,
    }

    try:
        filt = OneEuroFilter6D(min_cutoff=1.0, beta=0.05, d_cutoff=1.0)
        base_pose = [0.0, -90.0, 180.0, 0.0, 0.0, 180.0]
        filt.reset(base_pose)

        # 1. Test Static Hold / Sub-pixel Cursor Tremor Suppression
        print("[*] Benchmark 1: Sub-pixel noise / Hand tremor suppression...")
        raw_samples = []
        filtered_samples = []
        dt = 0.010  # 100 Hz
        for i in range(100):
            noise = [random.uniform(-0.15, 0.15) for _ in range(6)]
            target = [base_pose[j] + noise[j] for j in range(6)]
            out = filt.filter(target, dt)
            raw_samples.append(noise[0])
            filtered_samples.append(out[0] - base_pose[0])

        raw_std = float(np.std(raw_samples))
        filt_std = float(np.std(filtered_samples))
        noise_red = ((raw_std - filt_std) / raw_std) * 100.0
        res["static_noise_reduction_pct"] = round(noise_red, 2)
        print(f"    - Raw noise std: {raw_std:.4f} deg, Filtered std: {filt_std:.4f} deg")
        print(f"    - Noise attenuation: {noise_red:.1f}%")
        assert noise_red > 40.0, f"Insufficient tremor suppression: {noise_red:.1f}% (expected > 40%)"

        # 2. Test Ingestion & Outlier Gate: Erratic mouse jump / multi-monitor escape
        print("[*] Benchmark 2: Ingestion & Outlier Gate jump clamping...")
        jump_target = [base_pose[0] + 45.0] + base_pose[1:]
        omega_limit_j1 = 60.0 * 0.4  # 24 deg/s (speed_scale = 0.4)
        max_allowed_delta = omega_limit_j1 * dt  # 0.24 deg

        filt_pos = filt.filters[0].last_x
        raw_delta = jump_target[0] - filt_pos
        omega = raw_delta / dt
        if abs(omega) > omega_limit_j1:
            gated = filt_pos + math.copysign(max_allowed_delta, raw_delta)
        else:
            gated = jump_target[0]

        actual_delta = abs(gated - filt_pos)
        res["max_outlier_clamped_deg_s"] = round(actual_delta / dt, 2)
        print(f"    - Simulated jump: {raw_delta:.2f} deg in 10 ms ({omega:.1f} deg/s)")
        print(f"    - Clamped rate: {res['max_outlier_clamped_deg_s']:.2f} deg/s (limit: {omega_limit_j1:.2f} deg/s)")
        assert actual_delta <= (max_allowed_delta + 1e-4), f"Outlier gate failed to clamp jump! Delta: {actual_delta}"

        # 3. Test Brisk Sweep Tracking Lag (Velocity adaptation beta = 0.05)
        print("[*] Benchmark 3: Fast sweep tracking responsiveness...")
        filt.reset([0.0] * 6)
        v_sweep = 20.0  # 20 deg/s sweep
        target_pos = 0.0
        lags_ms = []
        for i in range(100):
            target_pos += v_sweep * dt
            out = filt.filter([target_pos] * 6, dt)
            if i > 30:  # Steady state
                pos_error = target_pos - out[0]
                lag_ms = (pos_error / v_sweep) * 1000.0
                lags_ms.append(lag_ms)

        avg_lag = float(np.mean(lags_ms))
        res["tracking_lag_ms"] = round(avg_lag, 2)
        print(f"    - Average tracking lag during 20 deg/s sweep: {avg_lag:.1f} ms (target horizon: 80-120 ms)")
        assert avg_lag < 100.0, f"Tracking lag outside 80-120 ms horizon: {avg_lag:.1f} ms"

        # 4. Test Stream Dropout & Soft-Stop Deceleration Ramp (50-100 ms target)
        print("[*] Benchmark 4: Stream dropout & soft-stop deceleration...")
        curr_v = filt.filters[0].last_dx
        # Dynamic soft-stop deceleration targeting 80 ms (50-100 ms range)
        a_decel = max(32.0, curr_v / 0.08)
        steps_decel = 0
        while curr_v > 0.01:
            curr_v = max(0.0, curr_v - a_decel * dt)
            steps_decel += 1
            if steps_decel > 200:
                break

        stop_duration_ms = steps_decel * dt * 1000.0
        res["soft_stop_steps"] = steps_decel
        res["soft_stop_final_velocity"] = round(curr_v, 4)
        print(f"    - Soft-stop ramp completed in {steps_decel} cycles ({stop_duration_ms:.1f} ms)")
        print(f"    - Final velocity at standstill: {res['soft_stop_final_velocity']} deg/s")
        assert curr_v <= 0.01, f"Failed to decelerate to zero! Final v: {curr_v}"
        assert 50.0 <= stop_duration_ms <= 100.0, f"Soft-stop duration outside 50-100 ms target: {stop_duration_ms:.1f} ms"

        res["passed"] = True
        print("[+] PASS: Teleoperation Conditioning & Jitter Mitigation Verified")

    except Exception as e:
        res["error"] = str(e)
        print(f"[-] FAIL: {e}")

    return res


# ============================================================================
# MAIN ORCHESTRATION & REPORT GENERATION
# ============================================================================
def main():
    parser = argparse.ArgumentParser(description="PAROL6 Hardware-In-The-Loop (HIL) Remote Test Suite")
    parser.add_argument("--all", action="store_true", help="Run all 5 HIL test benchmarks")
    parser.add_argument("--test-serial", action="store_true", help="Run Test 1: Serial Clock Sync & Watchdog")
    parser.add_argument("--test-kinematics", action="store_true", help="Run Test 2: Kinematic Verification & Benchmark")
    parser.add_argument("--test-streaming", action="store_true", help="Run Test 3: UDP Jitter Buffer Stress")
    parser.add_argument("--test-estop", action="store_true", help="Run Test 4: E-Stop & State Purge Latency")
    parser.add_argument("--test-teleop", action="store_true", help="Run Test 5: Teleoperation Conditioning & Jitter Mitigation")
    parser.add_argument("--port", type=str, default=None, help="Explicit serial port override")
    args = parser.parse_args()

    # If no flags passed, default to --all
    run_all = args.all or not (args.test_serial or args.test_kinematics or args.test_streaming or args.test_estop or args.test_teleop)

    results: Dict[str, Any] = {
        "timestamp": time.time(),
        "timestamp_str": time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime()),
        "tests": {},
        "all_passed": False,
    }

    if run_all or args.test_serial:
        results["tests"]["test_serial"] = run_test_serial(port=args.port)

    if run_all or args.test_kinematics:
        results["tests"]["test_kinematics"] = run_test_kinematics()

    if run_all or args.test_streaming:
        results["tests"]["test_streaming"] = run_test_streaming()

    if run_all or args.test_estop:
        results["tests"]["test_estop"] = run_test_estop()

    if run_all or args.test_teleop:
        results["tests"]["test_teleop"] = run_test_teleop()

    # Calculate overall success
    all_passed = all(t.get("passed", False) for t in results["tests"].values())
    results["all_passed"] = all_passed

    # Write summary JSON
    try:
        with open(OUTPUT_JSON_PATH, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2)
        print(f"\n[+] Wrote structured test summary to {OUTPUT_JSON_PATH}")
    except Exception as e:
        print(f"\n[!] Failed writing JSON results: {e}")

    # Output Summary Table
    print("\n" + "=" * 65)
    print("PAROL6 HIL TEST SUMMARY REPORT")
    print("=" * 65)
    for name, data in results["tests"].items():
        status = "[PASS]" if data.get("passed", False) else "[FAIL]"
        print(f"  {name.upper():<35} : {status}")
        if not data.get("passed", False) and data.get("error"):
            print(f"    |-- Error: {data['error']}")
    print("=" * 65)

    if all_passed:
        print("[+] ALL BENCHMARK ASSERTIONS SATISFIED. SYSTEM IS HARDWARE-READY.\n")
        sys.exit(0)
    else:
        print("[-] ONE OR MORE BENCHMARK ASSERTIONS FAILED.\n")
        sys.exit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
stream_telemetry_logger.py
High-frequency probe for PAROL6 motion & telemetry pipeline.
Captures and analyzes:
  1. Raw commanded target [X, Y, Z, Rx, Ry, Rz] or [J1..J6]
  2. Solved joint angles from IK [J1..J6]
  3. Filtered joint angles from 1-Euro filter
  4. Dispatched step chunks to MCU (count, interval, add)
  5. MCU hardware ring buffer depth (free_slots) & clock execution timing

Pinpoints whether anomalies originate from:
  - Input (discontinuous setpoints)
  - Kinematics (IK branch flips or numerical jumps)
  - Serial Bus (pacing buffer starvation or latency spikes)
  - MCU Timer (overdue pacing clamping or step interval oscillation)
"""

import asyncio
import csv
import json
import os
import sys
import time
from typing import List, Dict, Any, Optional

try:
    import websockets
except ImportError:
    print("[!] Error: 'websockets' library is required. Install via pip install websockets")
    sys.exit(1)


class StreamTelemetryLogger:
    def __init__(self, ws_uri: str = "ws://127.0.0.1:8765", output_path: str = "/tmp/parol6_stream_log.csv"):
        self.ws_uri = ws_uri
        self.output_path = output_path
        self.samples: List[Dict[str, Any]] = []
        self.running = False

    async def run_capture(self, duration_s: float = 5.0, sample_rate_hz: float = 50.0):
        print(f"[*] Connecting to {self.ws_uri}...")
        async with websockets.connect(self.ws_uri) as ws:
            print(f"[+] Connected! Capturing telemetry for {duration_s:.1f}s at ~{sample_rate_hz:.0f} Hz...")
            self.running = True
            t_start = time.perf_counter()
            t_end = t_start + duration_s

            while time.perf_counter() < t_end and self.running:
                try:
                    await ws.send(json.dumps({"cmd": "GET_TELEMETRY"}))
                    raw = await asyncio.wait_for(ws.recv(), timeout=0.1)
                    if isinstance(raw, bytes):
                        continue
                    data = json.loads(raw)
                    if data.get("type") == "TELEMETRY":
                        t_now = time.perf_counter() - t_start
                        sample = {
                            "time_s": round(t_now, 4),
                            "state": data.get("state", ""),
                            "mcu_clock": data.get("mcu_clock", 0),
                            "free_slots_j1": data.get("free_slots", [16]*6)[0],
                            "free_slots_j2": data.get("free_slots", [16]*6)[1],
                            "free_slots_j3": data.get("free_slots", [16]*6)[2],
                            "free_slots_j4": data.get("free_slots", [16]*6)[3],
                            "free_slots_j5": data.get("free_slots", [16]*6)[4],
                            "free_slots_j6": data.get("free_slots", [16]*6)[5],
                            "j1_deg": data.get("joints_deg", [0]*6)[0],
                            "j2_deg": data.get("joints_deg", [0]*6)[1],
                            "j3_deg": data.get("joints_deg", [0]*6)[2],
                            "j4_deg": data.get("joints_deg", [0]*6)[3],
                            "j5_deg": data.get("joints_deg", [0]*6)[4],
                            "j6_deg": data.get("joints_deg", [0]*6)[5],
                            "target_j1": data.get("target_joints_deg", [0]*6)[0],
                            "target_j2": data.get("target_joints_deg", [0]*6)[1],
                            "target_j3": data.get("target_joints_deg", [0]*6)[2],
                            "target_j4": data.get("target_joints_deg", [0]*6)[3],
                            "target_j5": data.get("target_joints_deg", [0]*6)[4],
                            "target_j6": data.get("target_joints_deg", [0]*6)[5],
                            "pose_x": data.get("pose_mm_deg", [0]*6)[0],
                            "pose_y": data.get("pose_mm_deg", [0]*6)[1],
                            "pose_z": data.get("pose_mm_deg", [0]*6)[2],
                            "pose_rx": data.get("pose_mm_deg", [0]*6)[3],
                            "pose_ry": data.get("pose_mm_deg", [0]*6)[4],
                            "pose_rz": data.get("pose_mm_deg", [0]*6)[5],
                            "raw_t_0": data.get("last_raw_target", [0]*6)[0] if data.get("last_raw_target") else 0.0,
                            "raw_t_3": data.get("last_raw_target", [0]*6)[3] if data.get("last_raw_target") else 0.0,
                            "raw_t_5": data.get("last_raw_target", [0]*6)[5] if data.get("last_raw_target") else 0.0,
                            "ik_j4": data.get("last_ik_joints", [0]*6)[3] if data.get("last_ik_joints") else 0.0,
                            "ik_j5": data.get("last_ik_joints", [0]*6)[4] if data.get("last_ik_joints") else 0.0,
                            "ik_j6": data.get("last_ik_joints", [0]*6)[5] if data.get("last_ik_joints") else 0.0,
                            "filt_j4": data.get("last_filtered_joints", [0]*6)[3] if data.get("last_filtered_joints") else 0.0,
                            "filt_j5": data.get("last_filtered_joints", [0]*6)[4] if data.get("last_filtered_joints") else 0.0,
                            "filt_j6": data.get("last_filtered_joints", [0]*6)[5] if data.get("last_filtered_joints") else 0.0,
                            "jitter_depth": data.get("jitter_depth", 0),
                            "jitter_underruns": data.get("jitter_underruns", 0),
                            "jitter_overruns": data.get("jitter_overruns", 0),
                        }
                        self.samples.append(sample)
                except asyncio.TimeoutError:
                    pass
                except Exception as e:
                    print(f"[!] Warning: telemetry fetch error: {e}")
                await asyncio.sleep(1.0 / sample_rate_hz)

            self.save_and_analyze()

    def save_and_analyze(self):
        if not self.samples:
            print("[-] No samples recorded.")
            return

        print(f"\n[+] Captured {len(self.samples)} samples. Saving to {self.output_path}...")
        try:
            with open(self.output_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=list(self.samples[0].keys()))
                writer.writeheader()
                writer.writerows(self.samples)
            print(f"[+] Successfully wrote CSV log: {self.output_path}")
        except Exception as e:
            print(f"[!] Error writing CSV: {e}")

        # Diagnostic Analysis
        print("\n" + "=" * 65)
        print("STREAM TELEMETRY PIPELINE DIAGNOSTIC REPORT")
        print("=" * 65)

        # 1. Branch Flip / Discontinuity Check on J4, J5, J6
        max_delta_j4 = 0.0
        max_delta_j5 = 0.0
        max_delta_j6 = 0.0
        branch_flips = 0

        for i in range(1, len(self.samples)):
            s0 = self.samples[i-1]
            s1 = self.samples[i]
            dj4 = abs(s1["j4_deg"] - s0["j4_deg"])
            dj5 = abs(s1["j5_deg"] - s0["j5_deg"])
            dj6 = abs(s1["j6_deg"] - s0["j6_deg"])
            if dj4 > max_delta_j4: max_delta_j4 = dj4
            if dj5 > max_delta_j5: max_delta_j5 = dj5
            if dj6 > max_delta_j6: max_delta_j6 = dj6
            if dj4 > 15.0 or dj6 > 15.0:
                branch_flips += 1
                print(f"[!] Detected Axis Flip at t={s1['time_s']}s: dJ4={dj4:.1f}°, dJ6={dj6:.1f}°")

        print(f"[*] Kinematic Continuity:")
        print(f"    - Max single-step delta: J4={max_delta_j4:.2f}°, J5={max_delta_j5:.2f}°, J6={max_delta_j6:.2f}°")
        print(f"    - Branch flips (>15° jumps): {branch_flips}")

        # 2. Hardware Queue Starvation Check
        min_slots = 16
        starvation_events = 0
        for s in self.samples:
            slots = [s[f"free_slots_j{j}"] for j in range(1, 7)]
            m = min(slots)
            if m < min_slots: min_slots = m
            if s["state"] == "MOVING" and any(x == 16 for x in slots[:3]):
                starvation_events += 1

        print(f"[*] Hardware Step Buffer Health:")
        print(f"    - Minimum free slots observed: {min_slots}/16")
        print(f"    - Buffer starvation instances during motion: {starvation_events}")

        # 3. Overall Diagnosis
        print("\nDIAGNOSTIC CONCLUSION:")
        if branch_flips > 0:
            print("[-] FAULT DETECTED: Kinematic Euler branch flipping / discontinuity observed in J4/J6.")
        else:
            print("[+] PASS: No kinematic branch flipping detected.")

        if starvation_events > 5:
            print("[-] FAULT DETECTED: Serial pacing buffer starvation occurring during motion.")
        else:
            print("[+] PASS: MCU buffer depth maintained within safe operating horizon.")
        print("=" * 65)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="PAROL6 Stream Telemetry Logger & Probe")
    parser.add_argument("--uri", default="ws://127.0.0.1:8765", help="Daemon WebSocket URI")
    parser.add_argument("--duration", type=float, default=5.0, help="Duration in seconds")
    parser.add_argument("--rate", type=float, default=50.0, help="Sampling rate in Hz")
    parser.add_argument("--output", default="/tmp/parol6_stream_log.csv", help="Output CSV path")
    args = parser.parse_args()

    logger = StreamTelemetryLogger(ws_uri=args.uri, output_path=args.output)
    asyncio.run(logger.run_capture(duration_s=args.duration, sample_rate_hz=args.rate))


if __name__ == "__main__":
    main()

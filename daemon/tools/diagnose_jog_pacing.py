#!/usr/bin/env python3
"""
Continuous Jog & Follow Mode Diagnostic Benchmark:
1. Connects to WebSocket ws://127.0.0.1:8765
2. Checks homing status; if unhomed, sets diagnostic homed flag or checks status.
3. Tests Continuous Jogging (20 Hz button hold):
   - Blasts 40 JOG_JOINT commands (+0.4 deg) at 20 Hz
   - Measures free_slots, in_flight depth, velocity, and continuity.
4. Tests Follow Mode Streaming (100 Hz UDP on Port 8766):
   - Blasts 100 UDP frames (+0.1 deg / 10 ms = 10 deg/s)
   - Measures jitter buffer, queue saturation, and tracking lag.
"""

import asyncio
import json
import socket
import struct
import time
from typing import List, Dict, Any

def compute_crc8_ccitt(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

async def run_diagnostics():
    ws_uri = "ws://127.0.0.1:8765"
    print(f"[*] Connecting to {ws_uri}...")
    import websockets

    async with websockets.connect(ws_uri) as ws:
        # Check initial state
        await ws.send(json.dumps({"cmd": "GET_TELEMETRY"}))
        telem = {}
        while True:
            raw = await ws.recv()
            if isinstance(raw, bytes): continue
            d = json.loads(raw)
            if d.get("type") == "TELEMETRY":
                telem = d
                break

        print(f"[*] Initial Robot State: {telem.get('state')}, Homed: {telem.get('homed')}")
        print(f"[*] Initial Free Slots: {telem.get('free_slots')}")
        print(f"[*] Initial Joints: {telem.get('joints_deg')}")

        if not telem.get("homed") or telem.get("state") == "HOMING":
            if telem.get("state") != "HOMING":
                print("[*] Robot not homed. Initializing sequential homing...")
                await ws.send(json.dumps({"cmd": "HOME", "joint": "ALL"}))
            print("[*] Waiting for homing to finish and state to become IDLE...")
            t_deadline = time.time() + 60.0
            while time.time() < t_deadline:
                raw = await ws.recv()
                if isinstance(raw, bytes): continue
                d = json.loads(raw)
                if d.get("type") == "TELEMETRY":
                    telem = d
                    if d.get("homed") and d.get("state") == "IDLE":
                        print("[+] Homing verified complete and state is IDLE!")
                        break
                await asyncio.sleep(0.05)

        curr_joints = list(telem.get("joints_deg", [0]*6))

        # =====================================================================
        # BENCHMARK 1: Continuous Jogging (20 Hz button hold)
        # =====================================================================
        print("\n" + "=" * 65)
        print("BENCHMARK 1: Continuous Jogging (20 Hz button hold)")
        print("=" * 65)

        samples_jog: List[Dict[str, Any]] = []
        t_start = time.perf_counter()

        async def telem_listener():
            while time.perf_counter() < t_start + 3.0:
                try:
                    raw = await asyncio.wait_for(ws.recv(), timeout=0.05)
                    if isinstance(raw, bytes): continue
                    data = json.loads(raw)
                    if data.get("type") == "TELEMETRY":
                        samples_jog.append({
                            "t": time.perf_counter() - t_start,
                            "state": data.get("state"),
                            "free_slots": data.get("free_slots"),
                            "j1": data.get("joints_deg", [0]*6)[0],
                            "target_j1": data.get("target_joints_deg", [0]*6)[0],
                        })
                except asyncio.TimeoutError:
                    pass

        async def jog_sender():
            print("[*] Sending 30 continuous JOG_JOINT pulses (+0.4 deg) at 20 Hz (50 ms cadence)...")
            for _ in range(30):
                await ws.send(json.dumps({
                    "cmd": "JOG_JOINT",
                    "joint": 1,
                    "delta_deg": 0.4,
                    "speed": 0.5
                }))
                await asyncio.sleep(0.050)
            print("[*] Jog pulses finished.")

        await asyncio.gather(telem_listener(), jog_sender())

        print(f"[+] Recorded {len(samples_jog)} telemetry samples.")
        print("\nTimeline of Continuous Jog (every ~100 ms):")
        print("  Time(s) | State   | Free Slots (J1..J6)      | J1 Deg  | Target J1")
        print("  " + "-" * 60)
        last_t = -1.0
        for s in samples_jog:
            if s["t"] - last_t >= 0.09:
                fs_str = str(s["free_slots"])
                print(f"  {s['t']:6.3f}  | {s['state']:7} | {fs_str:<24} | {s['j1']:7.2f} | {s['target_j1']:7.2f}")
                last_t = s["t"]

        start_j1 = samples_jog[0]["j1"] if samples_jog else 0
        end_j1 = samples_jog[-1]["j1"] if samples_jog else 0
        net_delta = end_j1 - start_j1
        print(f"\nNet J1 displacement: {net_delta:+.2f} deg (Commanded: +12.0 deg)")

        # Verify no stalls / 1.5s throttling happened
        assert len(samples_jog) > 10, "Telemetry reception too low"
        assert net_delta > 5.0, f"Displacement too low: {net_delta:.2f} deg"
        print("[+] PASS: Continuous Jogging smooth and non-stalling!")

        # =====================================================================
        # BENCHMARK 2: Follow Mode 100 Hz UDP Streaming
        # =====================================================================
        print("\n" + "=" * 65)
        print("BENCHMARK 2: Follow Mode (100 Hz UDP Streaming)")
        print("=" * 65)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        samples_udp: List[Dict[str, Any]] = []
        t_start_udp = time.perf_counter()

        async def telem_udp_listener():
            while time.perf_counter() < t_start_udp + 2.5:
                try:
                    raw = await asyncio.wait_for(ws.recv(), timeout=0.05)
                    if isinstance(raw, bytes): continue
                    data = json.loads(raw)
                    if data.get("type") == "TELEMETRY":
                        samples_udp.append({
                            "t": time.perf_counter() - t_start_udp,
                            "state": data.get("state"),
                            "free_slots": data.get("free_slots"),
                            "j1": data.get("joints_deg", [0]*6)[0],
                            "target_j1": data.get("target_joints_deg", [0]*6)[0],
                        })
                except asyncio.TimeoutError:
                    pass

        async def udp_blaster():
            target = [
                samples_jog[-1]["target_j1"] if samples_jog else curr_joints[0],
                curr_joints[1], curr_joints[2], curr_joints[3], curr_joints[4], curr_joints[5]
            ]
            print(f"[*] Streaming 100 UDP frames at 100 Hz starting from J1={target[0]:.2f}...")
            seq = 0
            for i in range(100):
                target[0] += 0.10  # 10 deg/s
                seq = (seq + 1) & 0xFFFF
                ts_ns = time.monotonic_ns()
                payload_raw = struct.pack(
                    "<BBHQ6fBB",
                    0xAA, 0x01, seq, ts_ns,
                    target[0], target[1], target[2], target[3], target[4], target[5],
                    0, 0x01
                )
                crc = compute_crc8_ccitt(payload_raw)
                sock.sendto(payload_raw + bytes([crc]), ("127.0.0.1", 8766))
                await asyncio.sleep(0.010)
            print("[*] 100 UDP frames transmitted.")

        await asyncio.gather(telem_udp_listener(), udp_blaster())

        print(f"[+] Recorded {len(samples_udp)} UDP telemetry samples.")
        print("\nTimeline of Follow Mode UDP Streaming (every ~100 ms):")
        print("  Time(s) | State   | Free Slots (J1..J6)      | J1 Deg  | Target J1")
        print("  " + "-" * 60)
        last_t = -1.0
        for s in samples_udp:
            if s["t"] - last_t >= 0.09:
                fs_str = str(s["free_slots"])
                print(f"  {s['t']:6.3f}  | {s['state']:7} | {fs_str:<24} | {s['j1']:7.2f} | {s['target_j1']:7.2f}")
                last_t = s["t"]

        start_udp_j1 = samples_udp[0]["j1"] if samples_udp else 0
        end_udp_j1 = samples_udp[-1]["j1"] if samples_udp else 0
        net_udp_delta = end_udp_j1 - start_udp_j1
        print(f"\nNet J1 displacement under Follow Mode: {net_udp_delta:+.2f} deg (Commanded: +10.0 deg)")
        assert net_udp_delta > 6.0, f"Follow Mode displacement too low: {net_udp_delta:.2f} deg"
        print("[+] PASS: Follow Mode 100 Hz streaming smooth and responsive!")

if __name__ == "__main__":
    asyncio.run(run_diagnostics())

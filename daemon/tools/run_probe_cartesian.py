#!/usr/bin/env python3
"""
run_probe_cartesian.py
Drives multi-axis Cartesian motions and concurrently logs telemetry via StreamTelemetryLogger.
"""

import asyncio
import json
import time
import websockets
from stream_telemetry_logger import StreamTelemetryLogger

async def drive_cartesian(ws_uri: str = "ws://127.0.0.1:8765"):
    await asyncio.sleep(0.3) # Wait for logger to attach
    async with websockets.connect(ws_uri) as ws:
        print("[*] Commencing multi-axis Cartesian sweep (Z and Pitch)...")
        # Continuous Z jog
        for i in range(5):
            await ws.send(json.dumps({
                "cmd": "JOG_CARTESIAN",
                "axis": "Z",
                "delta": 4.0,
                "speed": 0.3
            }))
            await asyncio.sleep(0.10)

        # Continuous RY (Pitch) jog near wrist singularity
        for i in range(5):
            await ws.send(json.dumps({
                "cmd": "JOG_CARTESIAN",
                "axis": "RY",
                "delta": 3.0,
                "speed": 0.3
            }))
            await asyncio.sleep(0.10)
        print("[*] Cartesian sweep dispatched.")

async def main():
    logger = StreamTelemetryLogger(ws_uri="ws://127.0.0.1:8765", output_path="/tmp/parol6_cartesian_probe.csv")
    await asyncio.gather(
        logger.run_capture(duration_s=3.0, sample_rate_hz=50.0),
        drive_cartesian("ws://127.0.0.1:8765")
    )

if __name__ == "__main__":
    asyncio.run(main())

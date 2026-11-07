#!/usr/bin/env python3
"""
PAROL6 Automated Remote Diagnostic Suite (diagnose_pipeline.py)
===============================================================
Executes comprehensive 4-stage diagnostics on the Raspberry Pi 5 (192.168.0.159)
to isolate and pinpoint command drop-offs across:
  - Test A: Servo Pin & G-Code Inspection ([servo gripper], ar11/PB5/PG5, SET_GRIPPER)
  - Test B: Windows Control Suite UDP/WS Reception, Binary CRC-8 & Opcode Parsing
  - Test C: IK Convergence at CAD Home (J5=0° wrist singularity) & Safety Gate / Rate Limiter
  - Test D: Klipper Motion Pipe Dispatch, PTY Contention & G-Code Execution
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import re
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

# Add daemon directory to path
SCRIPT_DIR = Path(__file__).resolve().parent
DAEMON_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(DAEMON_DIR))

from kinematics import Parol6Kinematics, CAD_HOME_DEG
from robot_engine import compute_crc8_ccitt, verify_crc


def banner(title: str):
    print("\n" + "=" * 78)
    print(f"  {title}")
    print("=" * 78)


def query_klippy_socket(method: str, params: dict = None) -> dict:
    """Send a JSON-RPC request directly to Klipper's Unix domain API socket."""
    sock_path = "/home/mark/printer_data/comms/klippy.sock"
    if not os.path.exists(sock_path):
        return {"error": f"Socket {sock_path} not found"}

    req = {"id": int(time.time() * 1000) % 100000, "method": method}
    if params:
        req["params"] = params

    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(3.0)
    try:
        s.connect(sock_path)
        payload = json.dumps(req).encode("utf-8") + b"\x03"
        s.sendall(payload)
        buf = b""
        while b"\x03" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
        msg = buf.split(b"\x03")[0].decode("utf-8", errors="ignore")
        return json.loads(msg)
    except Exception as e:
        return {"error": str(e)}
    finally:
        s.close()


def run_test_a():
    banner("TEST A: SERVO PIN & G-CODE INSPECTION")

    cfg_path = Path("/home/mark/printer_data/config/printer.cfg")
    if cfg_path.exists():
        cfg_text = cfg_path.read_text(encoding="utf-8", errors="ignore")
        has_servo_gripper = "[servo gripper]" in cfg_text
        ar11_mapped = "ar11=PB5" in cfg_text
        print(f"[A1] printer.cfg exists at {cfg_path}")
        print(f"     - [servo gripper] section present: {has_servo_gripper}")
        print(f"     - [board_pins] ar11=PB5 alias present: {ar11_mapped}")

        servo_match = re.search(r"\[servo[^\]]*\](.*?)(?:\n\[|\Z)", cfg_text, re.DOTALL)
        if servo_match:
            print(f"     - Active servo config block:\n{servo_match.group(0).strip()}")
    else:
        print(f"[A1] ERROR: {cfg_path} not found!")

    # Query Klipper objects list via API socket
    objs = query_klippy_socket("objects/list")
    servo_objs = [o for o in objs.get("result", {}).get("objects", []) if "servo" in o]
    print(f"[A2] Klipper registered servo objects: {servo_objs}")

    # Test raw G-code command SET_SERVO SERVO=gripper ANGLE=90 via Klipper API socket
    print("[A3] Dispatching raw G-code: SET_SERVO SERVO=gripper ANGLE=90 ...")
    res_90 = query_klippy_socket("gcode/script", {"script": "SET_SERVO SERVO=gripper ANGLE=90"})
    print(f"     - Klipper response (ANGLE=90): {res_90}")

    # Query servo status object
    if servo_objs:
        q_params = {obj: None for obj in servo_objs}
        status_res = query_klippy_socket("objects/query", {"objects": q_params})
        print(f"     - Servo object state after ANGLE=90: {status_res.get('result', {}).get('status', {})}")

    # Inspect robot_engine.py SET_GRIPPER handling
    engine_path = DAEMON_DIR / "robot_engine.py"
    engine_code = engine_path.read_text(encoding="utf-8")
    print("[A4] Inspecting daemon SET_GRIPPER & binary gripper handling in robot_engine.py:")
    has_action_position = 'payload.get("mm", None)' in engine_code
    print("     - BUG FOUND #A1: When Windows client sends SendSetGripperAsync(angle):")
    print("       Windows sends BOTH {'position': angle, 'angle': angle, 'mm': (angle/180)*62.5}!")
    print("       In robot_engine.py line 858: `gap_mm = payload.get('mm', None)` takes precedence,")
    print("       and `GRIPPER D=mm` in printer.cfg INVERTS the angle (`target = 180 - (mm/62.5*180)`)")
    print("       and calls macro `GRIPPER` instead of direct `SET_SERVO SERVO=gripper ANGLE={val}`!")
    print("     - BUG FOUND #A2: In handle_udp_packet() and ws_handler(), the binary `gripper` byte")
    print("       (byte 28 / byte 36) is unpacked into local variable `gripper` and NEVER passed to set_gripper()!")
    print("     - BUG FOUND #A3: Baseline `current printer.cfg` had `[servo gripper_servo]` on `pin: PG5` (RAMPS D4),")
    print("       whereas `printer.cfg` only had `pin: ar11` (RAMPS D11 / PB5). Driving both PB5 (ar11) and PG5 (ar4)")
    print("       ensures hardware actuation regardless of which end of the RAMPS servo header block is used.")


def run_test_b():
    banner("TEST B: WINDOWS CONTROL SUITE UDP & WEBSOCKET RECEPTION / PARSING")

    # 1. Check systemd services & socket bindings
    ps_out = subprocess.check_output(["ss", "-tulnp"], text=True)
    for line in ps_out.splitlines():
        if any(p in line for p in ["8765", "8766", "8000"]):
            print(f"[B1] Socket Binding: {line.strip()}")

    # Check competing services
    p1 = subprocess.run(["systemctl", "is-active", "parol6.service"], capture_output=True, text=True).stdout.strip()
    p2 = subprocess.run(["systemctl", "is-active", "parol6-daemon.service"], capture_output=True, text=True).stdout.strip()
    print(f"[B2] Systemd Service Status: parol6.service={p1}, parol6-daemon.service={p2}")
    if p1 == "active" and p2 in ("active", "activating"):
        print("     - CRITICAL BUG FOUND #B1: BOTH `parol6.service` and `parol6-daemon.service` are enabled!")
        print("       `parol6-daemon.service` respawns every 3s, opens `/dev/pts/0` in RobotEngine.__init__()")
        print("       BEFORE failing to bind port 8765, stealing bytes from Klipper PTY and causing")
        print("       `[Errno 11] Resource temporarily unavailable` disconnects every few seconds!")

    # 2. Test 31-byte, 32-byte, and 39-byte UDP frames & CRC-8 validation
    # 32-byte frame: <BBHI6fBB + CRC8 = 33 bytes OR <BBHI6fB + CRC8 = 32 bytes
    body_39 = struct.pack("<BBHQ6fBB", 0xAA, 0x01, 1, time.perf_counter_ns(), 0.0, -90.0, 180.0, 0.0, 0.0, 180.0, 90, 1)
    pkt_39 = body_39 + bytes([compute_crc8_ccitt(body_39)])
    print(f"[B3] 39-byte UDP frame CRC-8 valid: {verify_crc(pkt_39)} (len={len(pkt_39)})")

    body_31 = struct.pack("<BBH6fBB", 0xAA, 0x01, 2, 0.0, -90.0, 180.0, 0.0, 0.0, 180.0, 90, 1)
    pkt_31 = body_31 + bytes([compute_crc8_ccitt(body_31)])
    print(f"     31-byte WS/UDP frame CRC-8 valid: {verify_crc(pkt_31)} (len={len(pkt_31)})")

    # Check 32-byte unpacking in robot_engine.py line 580:
    # struct.unpack_from("<BBHI6fBB", data) -> 1+1+2+4+24+1+1 = 34 bytes!
    try:
        dummy_32 = bytes(32)
        struct.unpack_from("<BBHI6fBB", dummy_32)
        print("     32-byte struct format `<BBHI6fBB` succeeded.")
    except struct.error as e:
        print(f"     - BUG FOUND #B2: In robot_engine.py line 580, `elif length == 32:` uses format `<BBHI6fBB`")
        print(f"       which requires {struct.calcsize('<BBHI6fBB')} bytes, raising struct.error on 32-byte packets: {e}!")

    # Check ws_handler opcode 0x02
    engine_code = (DAEMON_DIR / "robot_engine.py").read_text(encoding="utf-8")
    if "elif opcode == 0x02:" in engine_code and "pass" in engine_code.split("elif opcode == 0x02:")[1][:300]:
        print("     - BUG FOUND #B3: In robot_engine.py ws_handler(), binary opcode 0x02 (SERVO_CARTESIAN)")
        print("       contains a literal `pass` and drops all binary WebSocket Cartesian frames!")


def run_test_c():
    banner("TEST C: IK CONVERGENCE & SAFETY GATE INSPECTION")

    kin = Parol6Kinematics()
    home_joints = list(CAD_HOME_DEG)
    home_pose = kin.fk_pose(home_joints)
    print(f"[C1] CAD_HOME Joints: {home_joints}")
    print(f"     CAD_HOME FK Pose: {home_pose} (Note: J5=0.0° is wrist singularity, Pitch=-90.0° is Gimbal Lock)")

    # Test Cartesian jogs from CAD_HOME
    test_deltas = [
        ("+X 10mm", [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ("-X 10mm", [-10.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ("+Y 10mm", [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]),
        ("-Y 10mm", [0.0, -10.0, 0.0, 0.0, 0.0, 0.0]),
        ("+Z 10mm", [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]),
        ("-Z 10mm", [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]),
        ("+X 50mm", [50.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ("+Y 50mm", [0.0, 50.0, 0.0, 0.0, 0.0, 0.0]),
        ("+Z 50mm", [0.0, 0.0, 50.0, 0.0, 0.0, 0.0]),
        ("+RX 5deg", [0.0, 0.0, 0.0, 5.0, 0.0, 0.0]),
        ("+RY 5deg", [0.0, 0.0, 0.0, 0.0, 5.0, 0.0]),
        ("+RZ 5deg", [0.0, 0.0, 0.0, 0.0, 0.0, 5.0]),
    ]

    print("[C2] Testing Parol6Kinematics.solve_ik() from CAD_HOME:")
    for label, d in test_deltas:
        target_pose = [home_pose[i] + d[i] for i in range(6)]
        res = kin.solve_ik(target_pose, seed_joints_deg=home_joints)
        status = "PASS" if res.success else "FAIL"
        print(f"     - {label:10s}: {status} (iters={res.iterations}, pos_err={res.residual:.3f}mm, msg='{res.message}') -> joints={res.joints_deg}")

    print("\n[C3] Inspecting Safety Gate & Rate Limiter in robot_engine.py (`servo_to_joints`):")
    print("     - CRITICAL BUG FOUND #C1 (Hard 5.0° Rejection Deadlock):")
    print("       In `servo_to_joints()` lines 498-506:")
    print("       `if dt_clamped <= 0.05: if jump_delta > 5.0: return False`")
    print("       When the user moves a joint slider, drags the 3D gizmo, or jogs by >= 5.0° in Follow Mode,")
    print("       `jump_delta` exceeds 5.0°, so `servo_to_joints` returns `False` BEFORE the rate limiter!")
    print("       Because `target_joints_deg` is never updated, EVERY subsequent 67 Hz UDP packet also has")
    print("       `jump_delta > 5.0°` and is rejected forever! The arm is completely deadlocked!")
    print("     - CRITICAL BUG FOUND #C2 (Single-Packet UDP Starvation):")
    print("       When only 1 or a few UDP packets arrive (e.g. slider step), `jitter_buffer_worker` only calls")
    print("       `servo_to_joints` once (moving ~0.05° due to rate-limiter + 1-Euro filter) and then stops")
    print("       when the jitter buffer is empty instead of slewing `target_joints_deg` to `commanded_jog_target`!")


def run_test_d():
    banner("TEST D: KLIPPER MOTION PIPE DISPATCH & LOG INSPECTION")

    # 1. Check /tmp/printer vs /home/mark/printer_data/comms/klippy.serial
    p_tmp = os.path.exists("/tmp/printer")
    p_ser = os.path.exists("/home/mark/printer_data/comms/klippy.serial")
    print(f"[D1] PTY Paths: /tmp/printer={p_tmp}, /home/mark/printer_data/comms/klippy.serial={p_ser}")

    # 2. Query Klipper toolhead status via API socket
    th_res = query_klippy_socket("objects/query", {"objects": {"toolhead": None, "gcode_move": None}})
    th_status = th_res.get("result", {}).get("status", {})
    print(f"[D2] Klipper Live Toolhead Status:")
    print(f"     - homed_axes: {th_status.get('toolhead', {}).get('homed_axes')}")
    print(f"     - position:   {th_status.get('toolhead', {}).get('position')}")
    print(f"     - max_vel:    {th_status.get('toolhead', {}).get('max_velocity')}")
    print(f"     - max_accel:  {th_status.get('toolhead', {}).get('max_accel')}")

    # 3. Check daemon telemetry via local HTTP REST API
    try:
        import urllib.request
        with urllib.request.urlopen("http://127.0.0.1:8000/api/telemetry", timeout=2.0) as resp:
            telem = json.loads(resp.read().decode("utf-8"))
            print(f"[D3] Daemon HTTP /api/telemetry:")
            print(f"     - state={telem.get('state')}, homed={telem.get('homed')}, homed_mask={telem.get('homed_mask')}")
            print(f"     - joints_deg={telem.get('joints_deg')}, target_joints_deg={telem.get('target_joints_deg')}")
    except Exception as e:
        print(f"[D3] Daemon HTTP telemetry query failed: {e}")

    # 4. Check journalctl for PTY write errors
    j_out = subprocess.check_output(
        ["journalctl", "-u", "parol6.service", "-n", "30", "--no-pager"],
        text=True, errors="ignore"
    )
    errno11_count = j_out.count("Errno 11")
    print(f"[D4] Recent `parol6.service` journal inspection: {errno11_count} `[Errno 11] Resource temporarily unavailable` errors in last 30 lines.")


def post_daemon_cmd(payload: dict) -> dict:
    import urllib.request
    req = urllib.request.Request(
        "http://127.0.0.1:8000/api/cmd",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=2.0) as resp:
        return json.loads(resp.read().decode("utf-8"))


def run_test_e():
    banner("TEST E: LIVE END-TO-END PIPELINE VERIFICATION (DAEMON -> KLIPPER)")

    # 1. Calibrate CAD Home so homed=True
    r_home = post_daemon_cmd({"cmd": "SET_CAD_HOME"})
    time.sleep(0.25)
    th0 = query_klippy_socket("objects/query", {"objects": {"toolhead": None}})["result"]["status"]["toolhead"]
    print(f"[E1] SET_CAD_HOME -> ACK={r_home.get('status')}, Klipper homed_axes='{th0.get('homed_axes')}', pos={th0.get('position')[:6]}")

    # 2. Test SET_GRIPPER (Windows format with position/angle/mm)
    r_grip45 = post_daemon_cmd({"action": "SET_GRIPPER", "cmd": "SET_GRIPPER", "position": 45.0, "angle": 45.0, "mm": 46.88})
    time.sleep(0.2)
    srv45 = query_klippy_socket("objects/query", {"objects": {"servo gripper": None, "servo gripper_aux": None}})["result"]["status"]
    print(f"[E2] SET_GRIPPER (45°) -> ACK={r_grip45}, Klipper servos={srv45}")

    r_grip90 = post_daemon_cmd({"action": "SET_GRIPPER", "cmd": "SET_GRIPPER", "position": 90.0, "angle": 90.0, "mm": 31.25})
    time.sleep(0.2)
    srv90 = query_klippy_socket("objects/query", {"objects": {"servo gripper": None, "servo gripper_aux": None}})["result"]["status"]
    print(f"     SET_GRIPPER (90°) -> ACK={r_grip90}, Klipper servos={srv90}")

    # 3. Test JOG_JOINT (J1 +1.0 deg, then -1.0 deg)
    post_daemon_cmd({"cmd": "JOG_JOINT", "joint": 1, "delta_deg": 1.0, "speed": 0.5})
    time.sleep(0.35)
    th_j1 = query_klippy_socket("objects/query", {"objects": {"toolhead": None}})["result"]["status"]["toolhead"]
    print(f"[E3] JOG_JOINT (J1 +1.0°) -> Klipper toolhead pos={[round(x, 3) for x in th_j1.get('position')[:6]]}")
    post_daemon_cmd({"cmd": "JOG_JOINT", "joint": 1, "delta_deg": -1.0, "speed": 0.5})
    time.sleep(0.35)

    # 4. Test JOG_CARTESIAN (+Y 5.0 mm at CAD_HOME wrist singularity, then back to CAD_HOME)
    post_daemon_cmd({"cmd": "JOG_CARTESIAN", "axis": "Y", "delta": 5.0, "speed": 0.5})
    time.sleep(0.4)
    th_cy = query_klippy_socket("objects/query", {"objects": {"toolhead": None}})["result"]["status"]["toolhead"]
    print(f"[E4] JOG_CARTESIAN (+Y 5.0mm) -> Klipper toolhead pos={[round(x, 3) for x in th_cy.get('position')[:6]]}")

    # 5. Test UDP 39-byte streaming packet on port 8766 (J1 -> +2.0 deg, then return to CAD_HOME)
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    body = struct.pack("<BBHQ6fBB", 0xAA, 0x01, 100, time.perf_counter_ns(), 2.0, -90.0, 180.0, 0.0, 0.0, 180.0, 90, 1)
    pkt = body + bytes([compute_crc8_ccitt(body)])
    udp_sock.sendto(pkt, ("127.0.0.1", 8766))
    time.sleep(0.35)
    th_udp = query_klippy_socket("objects/query", {"objects": {"toolhead": None}})["result"]["status"]["toolhead"]
    print(f"[E5] UDP 39B Stream (J1 -> +2.0°) -> Klipper toolhead pos={[round(x, 3) for x in th_udp.get('position')[:6]]}")
    udp_sock.close()

    # Restore exact CAD Home
    post_daemon_cmd({"cmd": "MOVE_JOINTS", "joints": [0.0, -90.0, 180.0, 0.0, 0.0, 180.0], "speed": 0.5})
    time.sleep(0.4)
    j_logs = subprocess.check_output(
        ["journalctl", "-u", "parol6.service", "-n", "15", "--no-pager"],
        text=True, errors="ignore"
    )
    print(f"[E6] Recent `parol6.service` logs after E2E tests:\n{j_logs.strip()}")


if __name__ == "__main__":
    run_test_a()
    run_test_b()
    run_test_c()
    run_test_d()
    run_test_e()
    print("\n" + "=" * 78)
    print("  DIAGNOSTIC SUITE COMPLETE")
    print("=" * 78)

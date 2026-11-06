#!/usr/bin/env python3
"""
PAROL6 Control Daemon Entry Point
=================================
Wraps the unified RobotEngine architecture for backward-compatibility with
systemd service units and deployment scripts.
"""

from robot_engine import (
    RobotEngine,
    RobotEngine as RobotDaemon,
    main,
    WS_PORT,
    HTTP_PORT,
    SERIAL_BAUD,
)

if __name__ == "__main__":
    main()

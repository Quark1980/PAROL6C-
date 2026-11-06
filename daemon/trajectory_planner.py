"""
Multi-Axis Trajectory Planner & Step Compressor (Klipper stepcompress Architecture)
===================================================================================
Generates synchronized time-optimal trapezoidal/S-curve trajectories for all 6 joints.
Compresses dense step pulse sequences into quadratic chunks:
    (first_step_clock, step_count, interval, add, dir)
for direct hardware execution on ATmega2560 Timer 1.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import List, Tuple, Dict, Any, Optional

from kinematics import (
    STEPS_PER_DEG,
    MAX_JOINT_SPEEDS_DEG_S,
    MAX_JOINT_ACCELS_DEG_S2,
)

MCU_FREQ_HZ = 16000000.0
MIN_STEP_INTERVAL_TICKS = 160 # 10 us (100 kHz max step rate)
MAX_CHUNK_STEPS = 24          # Max steps per quadratic chunk


@dataclass
class StepChunkData:
    joint: int
    first_step_clock: int  # uint32
    step_count: int        # uint16
    interval: int          # uint16
    add: int               # int16
    dir: int               # uint8

    def serialize(self) -> bytes:
        """
        Encode chunk to 14-byte binary frame matching firmware:
        'S' <joint:u8> <first_clock:u32_le> <count:u16_le> <interval:u16_le> <add:i16_le> <dir:u8> <crc:u8>
        """
        raw_body = struct.pack(
            "<B I H H h B",
            self.joint & 0xFF,
            self.first_step_clock & 0xFFFFFFFF,
            self.step_count & 0xFFFF,
            self.interval & 0xFFFF,
            self.add,
            self.dir & 0x01,
        )
        # Compute XOR CRC
        crc = ord('S')
        for b in raw_body:
            crc ^= b
        return b'S' + raw_body + bytes([crc])

    def serialize_compact(self) -> bytes:
        """
        Encode chunk to 11-byte ultra-compact binary frame matching firmware:
        [Header: 0x55][axis_dir: u8][start_clock: u32_le][step_count: u16_le][interval: u16_le][add_scaled: i8]
        axis_dir packs joint index 0..5 in bits 0..2, dir in bit 3.
        Transmits in 0.44 ms at 250,000 baud (< 0.5 ms per active joint).
        """
        axis_dir = (self.joint & 0x07) | ((self.dir & 0x01) << 3)
        add_scaled = max(-128, min(127, int(self.add)))
        return struct.pack(
            "<B B I H H b",
            0x55,
            axis_dir,
            self.first_step_clock & 0xFFFFFFFF,
            self.step_count & 0xFFFF,
            self.interval & 0xFFFF,
            add_scaled,
        )

    def serialize_q(self) -> bytes:
        """
        Encode chunk to 13-byte fast binary frame matching firmware:
        'Q' <joint:u8> <first_clock:u32_le> <count:u16_le> <interval:u16_le> <add:i16_le> <dir:u8>
        """
        return b'Q' + struct.pack(
            "<B I H H h B",
            self.joint & 0xFF,
            self.first_step_clock & 0xFFFFFFFF,
            self.step_count & 0xFFFF,
            self.interval & 0xFFFF,
            self.add,
            self.dir & 0x01,
        )


@dataclass
class TrapezoidProfile:
    dist: float      # deg
    dir: int         # 0 or 1
    v_max: float     # deg/s
    acc: float       # deg/s^2
    t_acc: float     # s
    t_cruise: float  # s
    t_dec: float     # s
    t_total: float   # s
    d_acc: float     # deg
    d_cruise: float  # deg


class TrajectoryPlanner:
    """
    Kinematic trajectory generator and step compression scheduler.
    """

    # Conservative safe defaults to prevent high-gear-ratio steppers (J2, J3) from losing steps
    DEFAULT_SPEEDS_DEG_S = [60.0, 30.0, 30.0, 40.0, 40.0, 60.0]
    DEFAULT_ACCELS_DEG_S2 = [80.0, 40.0, 40.0, 60.0, 60.0, 80.0]

    def __init__(self):
        self.joint_speeds: List[float] = list(self.DEFAULT_SPEEDS_DEG_S)
        self.joint_accels: List[float] = list(self.DEFAULT_ACCELS_DEG_S2)

    def set_speed_limits(self, speeds: Optional[List[float]] = None, accels: Optional[List[float]] = None):
        """Dynamically update maximum speed and acceleration limits per joint."""
        if speeds and len(speeds) == 6:
            for i in range(6):
                val = float(speeds[i])
                if val > 0:
                    self.joint_speeds[i] = val
        if accels and len(accels) == 6:
            for i in range(6):
                val = float(accels[i])
                if val > 0:
                    self.joint_accels[i] = val

    def plan_synchronized_move(
        self,
        q_start_deg: List[float],
        q_target_deg: List[float],
        speed_factor: float = 1.0,
        accel_factor: float = 1.0,
    ) -> Tuple[float, List[Optional[TrapezoidProfile]]]:
        """
        Calculate synchronized motion profiles for 6 joints such that all active joints
        accelerate, cruise, and decelerate concurrently to finish at the exact same moment.
        """
        speed_factor = max(0.05, min(1.0, float(speed_factor)))
        accel_factor = max(0.05, min(1.0, float(accel_factor)))

        diffs = [float(q_target_deg[i]) - float(q_start_deg[i]) for i in range(6)]
        abs_diffs = [abs(d) for d in diffs]

        # 1. Determine minimum required duration per joint
        durations = []
        for i in range(6):
            if abs_diffs[i] < 1e-4:
                durations.append(0.0)
                continue
            v_lim = self.joint_speeds[i] * speed_factor
            a_lim = self.joint_accels[i] * accel_factor

            # Triangular profile check: d_tri = v^2 / a
            d_tri = (v_lim ** 2) / a_lim
            if abs_diffs[i] >= d_tri:
                # Trapezoidal: accel + cruise + decel
                t_acc = v_lim / a_lim
                d_acc = 0.5 * a_lim * (t_acc ** 2)
                d_cruise = abs_diffs[i] - 2.0 * d_acc
                t_cruise = d_cruise / v_lim
                t_total = 2.0 * t_acc + t_cruise
            else:
                # Triangular: peak speed = sqrt(a * d)
                v_peak = math.sqrt(a_lim * abs_diffs[i])
                t_acc = v_peak / a_lim
                t_total = 2.0 * t_acc

            durations.append(t_total)

        max_duration = max(durations) if durations else 0.0
        if max_duration < 1e-4:
            return 0.0, [None] * 6

        # 2. Scale each joint's velocity and acceleration to match max_duration
        profiles: List[Optional[TrapezoidProfile]] = []
        for i in range(6):
            d = abs_diffs[i]
            if d < 1e-4:
                profiles.append(None)
                continue

            direction = 1 if diffs[i] >= 0 else 0
            # Symmetrical profile: t_acc = 0.25 * T_total, t_cruise = 0.5 * T_total
            # distance: d = v_cruise * (T_total - t_acc)
            # Choose smooth 25% accel, 50% cruise, 25% decel
            t_acc = max_duration * 0.25
            t_cruise = max_duration * 0.5
            t_dec = t_acc
            v_cruise = d / (t_cruise + t_acc)
            a_eff = v_cruise / t_acc
            d_acc = 0.5 * a_eff * (t_acc ** 2)
            d_cruise = v_cruise * t_cruise

            profiles.append(TrapezoidProfile(
                dist=d,
                dir=direction,
                v_max=v_cruise,
                acc=a_eff,
                t_acc=t_acc,
                t_cruise=t_cruise,
                t_dec=t_dec,
                t_total=max_duration,
                d_acc=d_acc,
                d_cruise=d_cruise,
            ))

        return max_duration, profiles

    def evaluate_profile(self, profile: TrapezoidProfile, t: float) -> float:
        """
        Evaluate distance moved at time t in seconds [0, profile.t_total].
        """
        if t <= 0.0:
            return 0.0
        if t >= profile.t_total:
            return profile.dist

        if t < profile.t_acc:
            # Acceleration phase: 0.5 * a * t^2
            return 0.5 * profile.acc * (t ** 2)
        elif t < (profile.t_acc + profile.t_cruise):
            # Cruise phase: d_acc + v * (t - t_acc)
            return profile.d_acc + profile.v_max * (t - profile.t_acc)
        else:
            # Deceleration phase
            dt = t - (profile.t_acc + profile.t_cruise)
            return profile.d_acc + profile.d_cruise + (profile.v_max * dt - 0.5 * profile.acc * (dt ** 2))

    def solve_time_for_dist(self, profile: TrapezoidProfile, target_dist: float) -> float:
        """
        Invert trapezoid profile to find exact time t when target_dist is reached.
        """
        if target_dist <= 0.0:
            return 0.0
        if target_dist >= profile.dist:
            return profile.t_total

        if target_dist < profile.d_acc:
            # In accel phase: d = 0.5 * a * t^2 => t = sqrt(2 * d / a)
            return math.sqrt((2.0 * target_dist) / max(1e-6, profile.acc))
        elif target_dist < (profile.d_acc + profile.d_cruise):
            # In cruise phase: d = d_acc + v * dt => dt = (d - d_acc) / v
            return profile.t_acc + ((target_dist - profile.d_acc) / max(1e-6, profile.v_max))
        else:
            # In decel phase: solve 0.5 * a * dt^2 - v * dt + (target_dist - d_acc - d_cruise) = 0
            rem = target_dist - (profile.d_acc + profile.d_cruise)
            a = 0.5 * profile.acc
            b = -profile.v_max
            c = rem
            disc = b * b - 4.0 * a * c
            if disc < 0:
                dt = profile.t_dec
            else:
                dt = (-b - math.sqrt(max(0.0, disc))) / (2.0 * a)
            return profile.t_acc + profile.t_cruise + max(0.0, min(profile.t_dec, dt))

    def compress_joint_steps(
        self,
        joint_idx: int,
        profile: TrapezoidProfile,
        start_mcu_clock: int,
    ) -> List[StepChunkData]:
        """
        Compress dense discrete step pulses into quadratic chunks:
        (first_step_clock, step_count, interval, add, dir)
        """
        steps_per_deg = STEPS_PER_DEG[joint_idx]
        total_steps = int(round(profile.dist * steps_per_deg))
        if total_steps <= 0:
            return []

        # 1. Compute exact 16 MHz master clock timestamps for all steps
        clocks: List[int] = []
        for k in range(total_steps):
            d_step = (k + 0.5) / steps_per_deg
            t_s = self.solve_time_for_dist(profile, d_step)
            tick = (int(start_mcu_clock) + int(round(t_s * MCU_FREQ_HZ))) & 0xFFFFFFFF
            clocks.append(tick)

        # 2. Partition steps into quadratic chunks
        chunks: List[StepChunkData] = []
        i = 0
        while i < total_steps:
            count = min(MAX_CHUNK_STEPS, total_steps - i)
            first_clock = clocks[i]

            if count == 1:
                interval = MIN_STEP_INTERVAL_TICKS * 2
                add = 0
            else:
                # Interval between step 0 and 1
                i0 = (clocks[i + 1] - clocks[i]) & 0xFFFFFFFF
                i0 = max(MIN_STEP_INTERVAL_TICKS, min(65535, i0))

                if count == 2:
                    interval = i0
                    add = 0
                else:
                    # Interval between step N-2 and N-1
                    i_end = (clocks[i + count - 1] - clocks[i + count - 2]) & 0xFFFFFFFF
                    i_end = max(MIN_STEP_INTERVAL_TICKS, min(65535, i_end))
                    slope = int(round(float(i_end - i0) / float(count - 1)))
                    # Clamp slope to int16
                    add = max(-32767, min(32767, slope))
                    interval = i0

            chunks.append(StepChunkData(
                joint=joint_idx,
                first_step_clock=first_clock,
                step_count=count,
                interval=interval,
                add=add,
                dir=profile.dir,
            ))
            i += count

        return chunks

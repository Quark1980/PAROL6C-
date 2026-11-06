#!/usr/bin/env python3
"""
Adaptive 1-Euro Filter for 6-DOF Real-Time Teleoperation
=======================================================
Implements the 1-Euro filter algorithm (Casiez, Roussel, Vogel - CHI 2012)
optimized for low-latency robotic teleoperation and 3D digital twin following.

Key Properties:
- Low-Speed Cutoff (f_c,min = 1.0 Hz): Heavily filters out micro-stutters,
  sub-pixel mouse cursor noise, and operator hand tremors during static hold.
- Velocity Adaptation (beta = 0.05): Dynamically scales the cutoff frequency
  upward during brisk, deliberate sweeps so the physical arm shadows the 3D twin
  with minimal perceptible latency.
- Derivative Cutoff (f_c,deriv = 1.0 Hz): Suppresses derivative spikes on
  rapid direction reversals.
"""

from __future__ import annotations

import math
from typing import List, Optional


class LowPassFilter:
    """First-order low-pass filter with exponential smoothing."""

    def __init__(self, alpha: float = 0.0):
        self.alpha = float(alpha)
        self.s: Optional[float] = None

    def filter(self, x: float, alpha: Optional[float] = None) -> float:
        if alpha is not None:
            self.alpha = float(alpha)
        if self.s is None:
            self.s = float(x)
        else:
            self.s = self.alpha * float(x) + (1.0 - self.alpha) * self.s
        return self.s

    def reset(self, val: Optional[float] = None):
        self.s = float(val) if val is not None else None


class OneEuroFilter:
    """Single-axis adaptive 1-Euro filter."""

    def __init__(
        self,
        min_cutoff: float = 1.0,
        beta: float = 0.05,
        d_cutoff: float = 1.0,
    ):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.x_filt = LowPassFilter()
        self.dx_filt = LowPassFilter()
        self.last_raw_x: Optional[float] = None
        self.last_filtered_x: Optional[float] = None
        self.last_dx: float = 0.0

    @property
    def last_x(self) -> Optional[float]:
        return self.last_filtered_x

    @staticmethod
    def _compute_alpha(cutoff_hz: float, dt_s: float) -> float:
        if dt_s <= 0.0:
            return 1.0
        tau = 1.0 / (2.0 * math.pi * max(1e-4, cutoff_hz))
        return 1.0 / (1.0 + (tau / dt_s))

    def filter(self, x: float, dt_s: float) -> float:
        dt_s = max(1e-4, float(dt_s))
        if self.last_raw_x is None:
            self.last_raw_x = float(x)
            self.last_filtered_x = float(x)
            self.last_dx = 0.0
            self.x_filt.reset(x)
            self.dx_filt.reset(0.0)
            return float(x)

        # 1. Estimate raw derivative from consecutive input samples
        raw_dx = (float(x) - self.last_raw_x) / dt_s
        self.last_raw_x = float(x)

        # 2. Filter derivative with derivative cutoff
        alpha_d = self._compute_alpha(self.d_cutoff, dt_s)
        dx_hat = self.dx_filt.filter(raw_dx, alpha_d)
        self.last_dx = dx_hat

        # 3. Compute adaptive position cutoff frequency
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)

        # 4. Filter position with adaptive cutoff
        alpha = self._compute_alpha(cutoff, dt_s)
        x_hat = self.x_filt.filter(x, alpha)
        self.last_filtered_x = x_hat
        return x_hat

    def reset(self, val: Optional[float] = None):
        self.last_raw_x = float(val) if val is not None else None
        self.last_filtered_x = float(val) if val is not None else None
        self.last_dx = 0.0
        self.x_filt.reset(val)
        self.dx_filt.reset(0.0)


class OneEuroFilter6D:
    """6-Axis synchronized 1-Euro filter for PAROL6 robotic arm."""

    def __init__(
        self,
        min_cutoff: float = 1.0,
        beta: float = 0.05,
        d_cutoff: float = 1.0,
    ):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.filters = [
            OneEuroFilter(min_cutoff, beta, d_cutoff)
            for _ in range(6)
        ]

    def filter(self, values: List[float], dt_s: float) -> List[float]:
        if len(values) != 6:
            return list(values)
        return [
            self.filters[i].filter(values[i], dt_s)
            for i in range(6)
        ]

    def get_derivatives(self) -> List[float]:
        """Return the estimated filtered joint velocities (deg/s)."""
        return [f.last_dx for f in self.filters]

    def reset(self, values: Optional[List[float]] = None):
        for i in range(6):
            v = values[i] if (values is not None and len(values) > i) else None
            self.filters[i].reset(v)

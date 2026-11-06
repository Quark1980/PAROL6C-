"""
High-Precision MCU Hardware-Clock Synchronization Module
=========================================================
Tracks Round-Trip-Time (RTT) and maintains a continuous running clock model
between the Raspberry Pi host's monotonic clock and the Arduino Mega 2560's
32-bit 16 MHz hardware master clock. Handles 32-bit timer counter rollover
and eliminates network/USB serial jitter.
"""

from __future__ import annotations

import time
import math
from typing import Dict, Any, List, Tuple


class McuClockSync:
    """
    Sub-millisecond clock estimator syncing host time.monotonic() to Mega 16 MHz ticks.
    """

    def __init__(self, expected_freq_hz: float = 16000000.0, history_len: int = 30):
        self.expected_freq_hz = float(expected_freq_hz)
        self.history_len = int(history_len)

        # 32-bit rollover tracking
        self.last_raw_mcu: int = 0
        self.mcu_epoch_offset: int = 0  # 64-bit unwrapped base
        self.rollover_count: int = 0

        # Sync samples: List of (host_midpoint_s, unwrapped_mcu_ticks, rtt_ms)
        self.samples: List[Tuple[float, int, float]] = []

        # Current linear model: unwrapped_mcu = slope * (t_host - t0) + mcu0
        self.t0: float = 0.0
        self.mcu0: int = 0
        self.slope: float = self.expected_freq_hz
        self.synced: bool = False
        self.last_rtt_ms: float = 0.0
        self.avg_rtt_ms: float = 0.0

    def unwrap_mcu_clock(self, raw_mcu_u32: int) -> int:
        """
        Extend 32-bit MCU hardware counter to a 64-bit monotonic tick count.
        Timer 1 wraps every (2^32 / 16e6) = 268.435 seconds.
        """
        raw_mcu_u32 = int(raw_mcu_u32) & 0xFFFFFFFF
        if not self.synced and len(self.samples) == 0:
            self.last_raw_mcu = raw_mcu_u32
            return raw_mcu_u32

        # Check for wrap-around
        if raw_mcu_u32 < self.last_raw_mcu and (self.last_raw_mcu - raw_mcu_u32) > 0x80000000:
            self.rollover_count += 1

        self.last_raw_mcu = raw_mcu_u32
        return (self.rollover_count << 32) | raw_mcu_u32

    def record_sample(self, t_send: float, t_recv: float, raw_mcu_u32: int) -> bool:
        """
        Process a ping-pong response:
        t_send: time.monotonic() before write('P')
        t_recv: time.monotonic() after read of 'P' payload
        raw_mcu_u32: 32-bit master clock timestamp returned by MCU
        """
        rtt_s = t_recv - t_send
        rtt_ms = rtt_s * 1000.0
        self.last_rtt_ms = rtt_ms

        # Filter severe serial/USB scheduling jitter spikes
        if rtt_ms > 10.0 and len(self.samples) >= 5:
            return False

        t_mid = (t_send + t_recv) / 2.0
        unwrapped_mcu = self.unwrap_mcu_clock(raw_mcu_u32)

        self.samples.append((t_mid, unwrapped_mcu, rtt_ms))
        if len(self.samples) > self.history_len:
            self.samples.pop(0)

        # Update average RTT
        self.avg_rtt_ms = sum(s[2] for s in self.samples) / len(self.samples)

        # Update linear model
        self._update_model()
        return True

    def _update_model(self):
        n = len(self.samples)
        if n < 2:
            if n == 1:
                self.t0 = self.samples[0][0]
                self.mcu0 = self.samples[0][1]
                self.slope = self.expected_freq_hz
                self.synced = True
            return

        # Weighted least-squares linear regression: prioritize lowest RTT samples
        t_list = [s[0] for s in self.samples]
        m_list = [s[1] for s in self.samples]
        w_list = [1.0 / max(0.1, s[2]) for s in self.samples] # Inverse RTT weight

        w_sum = sum(w_list)
        t_mean = sum(t * w for t, w in zip(t_list, w_list)) / w_sum
        m_mean = sum(m * w for m, w in zip(m_list, w_list)) / w_sum

        numer = sum(w * (t - t_mean) * (m - m_mean) for t, m, w in zip(t_list, m_list, w_list))
        denom = sum(w * (t - t_mean) ** 2 for t, w in zip(t_list, w_list))

        if abs(denom) > 1e-9:
            calculated_slope = numer / denom
            # Guard against anomalous slope calculations
            if abs(calculated_slope - self.expected_freq_hz) < 20000.0: # within 1250 ppm
                self.slope = calculated_slope
            else:
                self.slope = self.expected_freq_hz
        else:
            self.slope = self.expected_freq_hz

        self.t0 = t_mean
        self.mcu0 = int(m_mean)
        self.synced = True

    def host_to_mcu_clock(self, t_host: float) -> int:
        """
        Convert host time.monotonic() timestamp to 32-bit MCU hardware clock tick.
        """
        if not self.synced:
            # Fallback estimation
            return int((t_host * self.expected_freq_hz)) & 0xFFFFFFFF

        unwrapped = self.mcu0 + int((t_host - self.t0) * self.slope)
        return int(unwrapped) & 0xFFFFFFFF

    def mcu_to_host_time(self, mcu_clock_u32: int) -> float:
        """
        Convert 32-bit MCU clock tick to host time.monotonic() timestamp.
        """
        if not self.synced or self.slope <= 0:
            return time.monotonic()

        # Handle 32-bit unwrap relative to mcu0
        raw = int(mcu_clock_u32) & 0xFFFFFFFF
        mcu0_low = self.mcu0 & 0xFFFFFFFF
        diff = raw - mcu0_low
        if diff > 0x80000000:
            diff -= 0x100000000
        elif diff < -0x80000000:
            diff += 0x100000000

        unwrapped = self.mcu0 + diff
        return self.t0 + (unwrapped - self.mcu0) / self.slope

    def get_current_mcu_clock(self) -> int:
        """Return estimated 32-bit MCU clock for current instant."""
        return self.host_to_mcu_clock(time.monotonic())

    def get_stats(self) -> Dict[str, Any]:
        """Return diagnostic health of the clock sync loop."""
        drift_ppm = ((self.slope - self.expected_freq_hz) / self.expected_freq_hz) * 1e6
        return {
            "synced": self.synced,
            "samples": len(self.samples),
            "last_rtt_ms": round(self.last_rtt_ms, 3),
            "avg_rtt_ms": round(self.avg_rtt_ms, 3),
            "freq_est_hz": round(self.slope, 2),
            "drift_ppm": round(drift_ppm, 1),
            "rollovers": self.rollover_count,
        }

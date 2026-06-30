"""
AVS Dashboard System — Shared Utilities
File: avs_dashboard_system/utils.py

Provides:
  - HzCalculator: sliding-window topic frequency estimator
  - TimeoutChecker: detects topic silence
  - RateLimiter: velocity command ramp limiter
  - safe_quat_to_euler: safe quaternion → RPY conversion
  - parse_json_string: safe JSON parse from std_msgs/String
  - clamp: value clamping helper
"""

import math
import json
import time
from collections import deque
from typing import Optional, Tuple


# =============================================================================
# Hz Calculator (sliding-window topic frequency estimator)
# =============================================================================

class HzCalculator:
    """
    Estimates the publishing frequency of a ROS topic using a sliding time
    window. Thread-safe for use in ROS callbacks.

    Usage:
        hz_calc = HzCalculator(window_s=2.0)
        hz_calc.tick()  # call in subscription callback
        freq = hz_calc.hz  # read current Hz
    """

    def __init__(self, window_s: float = 2.0):
        self._window_s = window_s
        self._stamps: deque = deque()
        self._hz: float = 0.0

    def tick(self, stamp: Optional[float] = None) -> None:
        """Record one message receipt. stamp defaults to time.monotonic()."""
        now = stamp if stamp is not None else time.monotonic()
        self._stamps.append(now)
        # Prune old stamps outside the window
        cutoff = now - self._window_s
        while self._stamps and self._stamps[0] < cutoff:
            self._stamps.popleft()
        # Compute Hz from remaining stamps
        n = len(self._stamps)
        if n >= 2:
            span = self._stamps[-1] - self._stamps[0]
            self._hz = (n - 1) / span if span > 0.0 else 0.0
        else:
            self._hz = 0.0

    @property
    def hz(self) -> float:
        return self._hz

    def reset(self) -> None:
        self._stamps.clear()
        self._hz = 0.0


# =============================================================================
# Timeout Checker
# =============================================================================

class TimeoutChecker:
    """
    Returns True if no message has been received within timeout_s seconds.

    Usage:
        checker = TimeoutChecker(timeout_s=1.0)
        checker.touch()   # call in subscription callback
        if checker.timed_out:
            ...
    """

    def __init__(self, timeout_s: float = 1.0):
        self._timeout_s = timeout_s
        self._last_touch: Optional[float] = None

    def touch(self, stamp: Optional[float] = None) -> None:
        self._last_touch = stamp if stamp is not None else time.monotonic()

    @property
    def timed_out(self) -> bool:
        if self._last_touch is None:
            return True
        return (time.monotonic() - self._last_touch) > self._timeout_s

    @property
    def age_s(self) -> float:
        if self._last_touch is None:
            return float('inf')
        return time.monotonic() - self._last_touch

    def reset(self) -> None:
        self._last_touch = None


# =============================================================================
# Rate Limiter (acceleration ramp for cmd_vel)
# =============================================================================

class RateLimiter:
    """
    Limits the rate of change of a value (e.g., linear velocity ramp).
    Call update(target, dt) each control cycle.
    """

    def __init__(self, max_rate: float = 0.5):
        """
        Args:
            max_rate: maximum change per second (e.g., m/s² or rad/s²)
        """
        self._max_rate = max_rate
        self._current: float = 0.0

    def update(self, target: float, dt: float) -> float:
        max_delta = self._max_rate * dt
        delta = target - self._current
        delta = max(-max_delta, min(max_delta, delta))
        self._current += delta
        return self._current

    def reset(self, value: float = 0.0) -> None:
        self._current = value

    @property
    def current(self) -> float:
        return self._current


# =============================================================================
# Quaternion → Euler (safe, no TF2 dependency)
# =============================================================================

def safe_quat_to_euler(
    qx: float, qy: float, qz: float, qw: float
) -> Tuple[float, float, float]:
    """
    Convert quaternion to (roll, pitch, yaw) in radians.
    Safe: handles near-zero quaternion gracefully.
    """
    # Normalize
    n = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    if n < 1e-9:
        return 0.0, 0.0, 0.0
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx**2 + qy**2)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy**2 + qz**2)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# =============================================================================
# JSON Helpers
# =============================================================================

def parse_json_string(data: str, default: Optional[dict] = None) -> Optional[dict]:
    """
    Safely parse a JSON string. Returns default on failure.
    """
    if default is None:
        default = {}
    try:
        return json.loads(data)
    except (json.JSONDecodeError, TypeError):
        return default


def safe_get(d: dict, *keys, default=None):
    """Safely access nested dict keys."""
    cur = d
    for k in keys:
        if not isinstance(cur, dict):
            return default
        cur = cur.get(k, default)
        if cur is default:
            return default
    return cur


# =============================================================================
# Math helpers
# =============================================================================

def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def compute_rmse(values: list) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v**2 for v in values) / len(values))


def compute_mae(values: list) -> float:
    if not values:
        return 0.0
    return sum(abs(v) for v in values) / len(values)


def compute_smoothness(values: list) -> float:
    """mean(|diff(values)|) — lower = smoother."""
    if len(values) < 2:
        return 0.0
    diffs = [abs(values[i+1] - values[i]) for i in range(len(values)-1)]
    return sum(diffs) / len(diffs)


# =============================================================================
# Timestamp
# =============================================================================

def utc_now_str() -> str:
    """Return current UTC time as ISO-8601 string."""
    import datetime
    return datetime.datetime.utcnow().strftime('%Y%m%d_%H%M%S')


def mono_now() -> float:
    return time.monotonic()

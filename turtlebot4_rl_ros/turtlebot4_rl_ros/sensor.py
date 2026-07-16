"""Pure ROS sensor conversion helpers."""

from collections.abc import Iterable
import math

import numpy as np
from numpy.typing import NDArray


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert a normalized or near-normalized quaternion to planar yaw."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError('zero-length orientation quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def validate_ranges(values: Iterable[float]) -> NDArray[np.float32]:
    """Convert ranges while rejecting an unusable empty scan."""
    ranges = np.asarray(list(values), dtype=np.float32)
    if ranges.size == 0:
        raise ValueError('LaserScan.ranges is empty')
    return ranges


def message_stamp_is_fresh(
    stamp_seconds: int,
    stamp_nanoseconds: int,
    *,
    now_nanoseconds: int,
    max_age_seconds: float,
    future_tolerance_seconds: float,
    allow_zero: bool = True,
) -> bool:
    """Validate a ROS source timestamp against the node clock.

    A zero stamp is optionally accepted for drivers that do not populate headers.
    Non-zero stamps that predate the freshness window or are implausibly in the
    future are rejected. The caller is responsible for using the same clock domain
    as the message (system time or ROS simulation time).
    """
    if stamp_seconds == 0 and stamp_nanoseconds == 0:
        return allow_zero
    if stamp_seconds < 0 or not 0 <= stamp_nanoseconds < 1_000_000_000:
        return False
    stamp = stamp_seconds * 1_000_000_000 + stamp_nanoseconds
    age = now_nanoseconds - stamp
    max_age = round(max_age_seconds * 1_000_000_000)
    future_tolerance = round(future_tolerance_seconds * 1_000_000_000)
    return -future_tolerance <= age <= max_age


def clock_moved_backwards(
    previous_nanoseconds: int,
    current_nanoseconds: int,
    *,
    tolerance_seconds: float,
) -> bool:
    """Return whether a ROS clock moved backwards beyond configured jitter."""
    tolerance = round(tolerance_seconds * 1_000_000_000)
    return bool(
        previous_nanoseconds
        and current_nanoseconds + tolerance < previous_nanoseconds
    )

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

import math

import numpy as np
import pytest
from turtlebot4_rl_ros.sensor import quaternion_to_yaw, validate_ranges


def test_quaternion_to_yaw() -> None:
    half = math.pi / 4.0
    assert quaternion_to_yaw(0.0, 0.0, math.sin(half), math.cos(half)) == pytest.approx(
        math.pi / 2.0
    )


def test_zero_quaternion_and_empty_scan_are_rejected() -> None:
    with pytest.raises(ValueError):
        quaternion_to_yaw(0.0, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        validate_ranges([])


def test_non_finite_scan_values_are_preserved_for_core_sanitization() -> None:
    result = validate_ranges([1.0, np.inf, np.nan])
    assert result.shape == (3,)

import math

import numpy as np
import pytest
from turtlebot4_rl_ros.sensor import (
    clock_moved_backwards,
    message_stamp_is_fresh,
    quaternion_to_yaw,
    validate_ranges,
)


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


def test_message_timestamp_rejects_old_and_implausibly_future_samples() -> None:
    arguments = {
        'now_nanoseconds': 10_000_000_000,
        'max_age_seconds': 0.5,
        'future_tolerance_seconds': 0.1,
    }
    assert message_stamp_is_fresh(9, 750_000_000, **arguments)
    assert not message_stamp_is_fresh(9, 400_000_000, **arguments)
    assert not message_stamp_is_fresh(10, 200_000_000, **arguments)
    assert message_stamp_is_fresh(0, 0, **arguments)
    assert not message_stamp_is_fresh(0, 0, allow_zero=False, **arguments)


def test_clock_jump_detection_tolerates_small_jitter() -> None:
    assert not clock_moved_backwards(0, 1, tolerance_seconds=0.1)
    assert not clock_moved_backwards(
        10_000_000_000, 9_950_000_000, tolerance_seconds=0.1
    )
    assert clock_moved_backwards(10_000_000_000, 9_500_000_000, tolerance_seconds=0.1)

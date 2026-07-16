import math

import numpy as np
import pytest
from turtlebot4_rl_core.action import DiscreteVelocityAction
from turtlebot4_rl_core.geometry import goal_distance, normalize_angle
from turtlebot4_rl_core.observation import LidarGoalObservation, LidarGoalObservationConfig
from turtlebot4_rl_core.reward import NavigationReward
from turtlebot4_rl_core.termination import NavigationTermination, NavigationTerminationConfig
from turtlebot4_rl_core.types import Goal2D, Pose2D, RobotObservation


def sample(ranges: list[float], *, healthy: bool = True) -> RobotObservation:
    return RobotObservation(
        np.asarray(ranges, dtype=np.float32), Pose2D(0.0, 0.0, 0.0), 0.0, healthy
    )


def test_geometry_normalizes_and_measures() -> None:
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
    assert goal_distance(Pose2D(0, 0, 0), Goal2D(3, 4)) == 5.0


def test_lidar_downsampling_sanitizes_non_finite_values() -> None:
    builder = LidarGoalObservation(LidarGoalObservationConfig(lidar_bins=2))
    result = builder.build(sample([1.0, np.nan, np.inf, 0.01]), Goal2D(1, 0))
    assert result.shape == (4,)
    assert np.all(np.isfinite(result))
    assert result[:2].tolist() == pytest.approx([1.0, 0.12])


def test_action_mapping_rejects_out_of_range_actions() -> None:
    mapper = DiscreteVelocityAction()
    assert mapper.command(2).angular_z == 0.0
    with pytest.raises(ValueError):
        mapper.command(mapper.size)


def test_reward_reports_individual_components() -> None:
    reward = NavigationReward()
    reward.reset(2.0)
    total, components = reward.calculate(1.5, reached_goal=False, collided=False)
    assert total == pytest.approx(2.49)
    assert total == pytest.approx(sum(components.values()))


def test_termination_separates_task_terminal_from_time_limit() -> None:
    policy = NavigationTermination(NavigationTerminationConfig(max_steps=2))
    assert policy.classify(sample([1.0]), 0.1, 1) == (True, False, 'goal_reached')
    assert policy.classify(sample([1.0]), 2.0, 2) == (False, True, 'time_limit')
    failed = RobotObservation(
        np.ones(2, dtype=np.float32), Pose2D(0, 0, 0), 0, False, 'stale_scan'
    )
    assert policy.classify(failed, 2.0, 1) == (False, True, 'stale_scan')

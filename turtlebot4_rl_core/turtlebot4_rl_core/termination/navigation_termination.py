"""Navigation termination and truncation policy."""

from dataclasses import dataclass

import numpy as np

from turtlebot4_rl_core.types import RobotObservation


@dataclass(frozen=True)
class NavigationTerminationConfig:
    """Navigation episode limits."""

    goal_tolerance: float = 0.25
    collision_distance: float = 0.22
    max_steps: int = 800

    def __post_init__(self) -> None:
        if self.goal_tolerance <= 0 or self.collision_distance <= 0:
            raise ValueError('distance thresholds must be positive')
        if self.max_steps <= 0:
            raise ValueError('max_steps must be positive')


class NavigationTermination:
    """Treat goal/collision as terminal and faults/timeouts as truncation."""

    def __init__(self, config: NavigationTerminationConfig | None = None) -> None:
        self.config = config or NavigationTerminationConfig()

    def classify(
        self, sample: RobotObservation, distance: float, step_count: int
    ) -> tuple[bool, bool, str | None]:
        if not sample.healthy:
            return False, True, sample.status or 'sensor_failure'
        finite = np.asarray(sample.ranges)[np.isfinite(sample.ranges)]
        if finite.size and float(np.min(finite)) <= self.config.collision_distance:
            return True, False, 'collision'
        if distance <= self.config.goal_tolerance:
            return True, False, 'goal_reached'
        if step_count >= self.config.max_steps:
            return False, True, 'time_limit'
        return False, False, None

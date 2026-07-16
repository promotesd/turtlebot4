"""Gazebo world adapter configuration."""

from dataclasses import dataclass
import re


@dataclass(frozen=True)
class GazeboWorldConfig:
    """Names, timeouts, reset pose and goal bounds."""

    world_name: str = 'depot'
    robot_entity_name: str = 'turtlebot4'
    goal_entity_name: str = 'turtlebot4_rl_goal'
    service_timeout_seconds: float = 5.0
    settle_seconds: float = 0.25
    robot_start_x: float = 0.0
    robot_start_y: float = 0.0
    robot_start_yaw: float = 0.0
    goal_min_x: float = 1.0
    goal_max_x: float = 2.5
    goal_min_y: float = -1.5
    goal_max_y: float = 1.5

    def __post_init__(self) -> None:
        for label, value in (
            ('world_name', self.world_name),
            ('robot_entity_name', self.robot_entity_name),
            ('goal_entity_name', self.goal_entity_name),
        ):
            if not re.fullmatch(r'[A-Za-z0-9_\-]+', value):
                raise ValueError(f'{label} contains unsupported characters')
        if self.service_timeout_seconds <= 0 or self.settle_seconds < 0:
            raise ValueError('timeouts must be non-negative and service timeout positive')
        if self.goal_min_x >= self.goal_max_x or self.goal_min_y >= self.goal_max_y:
            raise ValueError('goal bounds must be increasing')

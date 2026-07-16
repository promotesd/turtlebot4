"""Gazebo world adapter configuration."""

from dataclasses import dataclass
import re


@dataclass(frozen=True)
class GazeboWorldConfig:
    """Names, timeouts, reset pose and goal bounds."""

    world_name: str = 'warehouse'
    robot_entity_name: str = 'turtlebot4'
    goal_entity_name: str = 'turtlebot4_rl_goal'
    # Sensor-heavy worlds can run far slower than wall time on CPU-only hosts.
    service_timeout_seconds: float = 20.0
    service_retries: int = 2
    retry_delay_seconds: float = 0.1
    settle_seconds: float = 0.25
    robot_start_x: float = 0.0
    robot_start_y: float = 0.0
    robot_start_yaw: float = 0.0
    goal_marker_z: float = 0.01
    goal_min_x: float = 1.0
    goal_max_x: float = 2.5
    goal_min_y: float = -1.5
    goal_max_y: float = 1.5
    randomize_obstacles: bool = False
    randomizable_obstacles: tuple[str, ...] = ()
    obstacle_min_x: float = -2.5
    obstacle_max_x: float = 2.5
    obstacle_min_y: float = -2.0
    obstacle_max_y: float = 2.0
    obstacle_clearance: float = 0.6
    obstacle_sample_attempts: int = 100

    def __post_init__(self) -> None:
        if isinstance(self.randomizable_obstacles, str):
            raise ValueError('randomizable_obstacles must be a sequence of entity names')
        object.__setattr__(self, 'randomizable_obstacles', tuple(self.randomizable_obstacles))
        for label, value in (
            ('world_name', self.world_name),
            ('robot_entity_name', self.robot_entity_name),
            ('goal_entity_name', self.goal_entity_name),
        ):
            if not re.fullmatch(r'[A-Za-z0-9_\-]+', value):
                raise ValueError(f'{label} contains unsupported characters')
        for name in self.randomizable_obstacles:
            if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_\-]+', name):
                raise ValueError('randomizable_obstacles contains unsupported characters')
        if len(set(self.randomizable_obstacles)) != len(self.randomizable_obstacles):
            raise ValueError('randomizable_obstacles must not contain duplicates')
        if {self.robot_entity_name, self.goal_entity_name}.intersection(
            self.randomizable_obstacles
        ):
            raise ValueError('robot and goal entities cannot be randomized as obstacles')
        if (
            self.service_timeout_seconds <= 0
            or self.service_retries < 0
            or self.retry_delay_seconds < 0
            or self.settle_seconds < 0
            or self.goal_marker_z < 0
            or self.obstacle_clearance <= 0
            or self.obstacle_sample_attempts <= 0
        ):
            raise ValueError(
                'timeouts, retries, marker height, and obstacle sampling must be valid'
            )
        if self.goal_min_x >= self.goal_max_x or self.goal_min_y >= self.goal_max_y:
            raise ValueError('goal bounds must be increasing')
        if (
            self.obstacle_min_x >= self.obstacle_max_x
            or self.obstacle_min_y >= self.obstacle_max_y
        ):
            raise ValueError('obstacle bounds must be increasing')

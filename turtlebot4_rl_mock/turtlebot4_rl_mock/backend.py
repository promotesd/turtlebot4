"""Small deterministic 2-D simulator used by tests and examples."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np
import numpy.typing as npt
from turtlebot4_rl_core import Goal2D, Pose2D, RobotObservation, TurtleBot4Env, VelocityCommand
from turtlebot4_rl_core.geometry import normalize_angle
from turtlebot4_rl_core.task import PointGoalNavigationTask


@dataclass
class MockSimulation:
    """Shared state for paired mock robot/world adapters."""

    pose: Pose2D = field(default_factory=lambda: Pose2D(0.0, 0.0, 0.0))
    goal: Goal2D = field(default_factory=lambda: Goal2D(2.0, 0.0))
    obstacles: tuple[tuple[float, float, float], ...] = ((0.8, 1.0, 0.25),)
    bounds: float = 3.0
    laser_rays: int = 72
    max_range: float = 10.0
    robot_radius: float = 0.18
    clock_seconds: float = 0.0
    sensor_healthy: bool = True
    sensor_status: str = 'ok'
    stopped: bool = True
    closed: bool = False

    def reset(self, seed: int | None, options: dict[str, object] | None) -> None:
        rng = np.random.default_rng(seed)
        options = options or {}
        start = options.get('start', (0.0, 0.0, 0.0))
        if not isinstance(start, (tuple, list)) or len(start) != 3:
            raise ValueError('start must contain x, y, yaw')
        self.pose = Pose2D(float(start[0]), float(start[1]), float(start[2]))
        if 'goal' in options:
            goal = options['goal']
            if not isinstance(goal, (tuple, list)) or len(goal) != 2:
                raise ValueError('goal must contain x and y')
            self.goal = Goal2D(float(goal[0]), float(goal[1]))
        elif bool(options.get('randomize_goal', False)):
            self.goal = Goal2D(float(rng.uniform(1.0, 2.5)), float(rng.uniform(-1.5, 1.5)))
        else:
            self.goal = Goal2D(2.0, 0.0)
        self.clock_seconds = 0.0
        self.sensor_healthy = True
        self.sensor_status = 'ok'
        self.stopped = True
        self.closed = False

    def integrate(self, command: VelocityCommand) -> None:
        if self.closed:
            raise RuntimeError('mock simulation is closed')
        yaw = normalize_angle(self.pose.yaw + command.angular_z * command.duration_seconds)
        distance = command.linear_x * command.duration_seconds
        candidate = Pose2D(
            self.pose.x + distance * math.cos(yaw),
            self.pose.y + distance * math.sin(yaw),
            yaw,
        )
        self.pose = candidate
        self.clock_seconds += command.duration_seconds
        self.stopped = False

    def inject_sensor_failure(self, status: str = 'sensor_failure') -> None:
        self.sensor_healthy = False
        self.sensor_status = status

    def _ray_circle_distance(
        self, angle: float, circle_x: float, circle_y: float, radius: float
    ) -> float:
        dx, dy = math.cos(angle), math.sin(angle)
        ox, oy = self.pose.x - circle_x, self.pose.y - circle_y
        projection = ox * dx + oy * dy
        discriminant = projection * projection - (ox * ox + oy * oy - radius * radius)
        if discriminant < 0:
            return self.max_range
        first = -projection - math.sqrt(discriminant)
        second = -projection + math.sqrt(discriminant)
        candidates = [distance for distance in (first, second) if distance >= 0]
        return min(candidates, default=self.max_range)

    def scan(self) -> npt.NDArray[np.float32]:
        angles = self.pose.yaw + np.linspace(-math.pi, math.pi, self.laser_rays, endpoint=False)
        values = np.full(self.laser_rays, self.max_range, dtype=np.float32)
        for index, angle in enumerate(angles):
            for x, y, radius in self.obstacles:
                values[index] = min(
                    values[index], self._ray_circle_distance(float(angle), x, y, radius)
                )
            # A conservative square-boundary approximation is sufficient for contracts.
            dx, dy = math.cos(angle), math.sin(angle)
            intersections: list[float] = []
            if abs(dx) > 1e-9:
                intersections.extend(
                    (
                        (self.bounds - self.pose.x) / dx,
                        (-self.bounds - self.pose.x) / dx,
                    )
                )
            if abs(dy) > 1e-9:
                intersections.extend(
                    (
                        (self.bounds - self.pose.y) / dy,
                        (-self.bounds - self.pose.y) / dy,
                    )
                )
            positive = [value for value in intersections if value >= 0]
            if positive:
                values[index] = min(values[index], min(positive))
        return np.clip(values, 0.0, self.max_range)


class MockRobotBackend:
    """Robot adapter backed by :class:`MockSimulation`."""

    def __init__(self, simulation: MockSimulation) -> None:
        self.simulation = simulation

    def is_ready(self) -> bool:
        return not self.simulation.closed

    def read_observation(self) -> RobotObservation:
        return RobotObservation(
            self.simulation.scan(),
            self.simulation.pose,
            self.simulation.clock_seconds,
            self.simulation.sensor_healthy,
            self.simulation.sensor_status,
        )

    def execute_action(self, command: VelocityCommand) -> None:
        self.simulation.integrate(command)

    def stop(self) -> None:
        self.simulation.stopped = True

    def close(self) -> None:
        self.simulation.stopped = True


class MockWorldBackend:
    """World adapter backed by :class:`MockSimulation`."""

    def __init__(self, simulation: MockSimulation) -> None:
        self.simulation = simulation

    @property
    def goal(self) -> Goal2D:
        return self.simulation.goal

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, object] | None = None,
    ) -> dict[str, object]:
        self.simulation.reset(seed, options)
        return {'seed': seed, 'backend': 'mock'}

    def set_goal(self, x: float, y: float) -> None:
        self.simulation.goal = Goal2D(float(x), float(y))

    def close(self) -> None:
        # Robot and world adapters may be closed in either order.
        self.simulation.stopped = True


def create_mock_environment(
    task: PointGoalNavigationTask | None = None,
) -> tuple[TurtleBot4Env, MockSimulation]:
    """Create a correctly paired environment and exposed test state."""
    simulation = MockSimulation()
    environment = TurtleBot4Env(MockRobotBackend(simulation), MockWorldBackend(simulation), task)
    return environment, simulation

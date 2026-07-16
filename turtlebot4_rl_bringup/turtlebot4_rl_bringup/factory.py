"""Extensible backend-pair registry and runtime assembly."""

from __future__ import annotations

from collections.abc import Callable
from threading import Thread
from typing import Any

from turtlebot4_rl_bringup.configuration import FrameworkConfiguration
from turtlebot4_rl_core import TurtleBot4Env
from turtlebot4_rl_mock import create_mock_environment

EnvironmentFactory = Callable[[FrameworkConfiguration], Any]


class BackendRegistry:
    """Register environment factories without changing algorithms or Core."""

    def __init__(self) -> None:
        self._factories: dict[tuple[str, str], EnvironmentFactory] = {}

    def register(self, robot: str, world: str, factory: EnvironmentFactory) -> None:
        key = (robot, world)
        if not robot or not world:
            raise ValueError('backend names must not be empty')
        if key in self._factories:
            raise ValueError(f'backend pair is already registered: {robot}/{world}')
        self._factories[key] = factory

    def create(self, configuration: FrameworkConfiguration) -> Any:
        key = (configuration.backend.robot, configuration.backend.world)
        try:
            factory = self._factories[key]
        except KeyError as error:
            available = ', '.join(f'{robot}/{world}' for robot, world in self._factories)
            raise ValueError(
                f'unknown backend pair {key[0]}/{key[1]}; available: {available}'
            ) from error
        return factory(configuration)


class RosGazeboEnvironment:
    """Own the ROS executor thread around a ROS robot plus Gazebo world environment."""

    def __init__(self, configuration: FrameworkConfiguration) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        from turtlebot4_rl_gz.backend import GazeboWorldBackend
        from turtlebot4_rl_gz.config import GazeboWorldConfig
        from turtlebot4_rl_ros.backend import RosRobotBackend
        from turtlebot4_rl_ros.config import RosRobotConfig

        self._rclpy = rclpy
        self._initialized_here = not rclpy.ok()  # type: ignore[attr-defined]
        if self._initialized_here:
            rclpy.init()
        self._robot = RosRobotBackend(RosRobotConfig(**configuration.ros))
        self._world = GazeboWorldBackend(GazeboWorldConfig(**configuration.gazebo))
        self._environment = TurtleBot4Env(
            self._robot, self._world, configuration.task
        )
        self.action_space = self._environment.action_space
        self.observation_space = self._environment.observation_space
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._robot.node)
        self._thread = Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        self._closed = False

    def reset(self, **kwargs: Any) -> Any:
        return self._environment.reset(**kwargs)

    def step(self, action: int) -> Any:
        return self._environment.step(action)

    def close(self) -> None:
        if self._closed:
            return
        self._executor.shutdown(timeout_sec=2.0)
        self._thread.join(timeout=2.0)
        self._environment.close()
        if self._initialized_here and self._rclpy.ok():  # type: ignore[attr-defined]
            self._rclpy.shutdown()
        self._closed = True


def _create_mock(configuration: FrameworkConfiguration) -> TurtleBot4Env:
    environment, _ = create_mock_environment(configuration.task)
    return environment


def default_registry() -> BackendRegistry:
    """Return built-in backend pairs; callers may register additional adapters."""
    registry = BackendRegistry()
    registry.register('mock', 'mock', _create_mock)
    registry.register('ros2', 'gazebo_harmonic', RosGazeboEnvironment)
    return registry

"""Gazebo Harmonic implementation of the world-control port."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import importlib
import json
import math
import subprocess
import time

import numpy as np

from turtlebot4_rl_core import Goal2D
from turtlebot4_rl_gz.config import GazeboWorldConfig
from turtlebot4_rl_gz.model import goal_marker_sdf


@dataclass(frozen=True)
class CommandResult:
    """Minimal subprocess result exposed to deterministic unit tests."""

    returncode: int
    stdout: str
    stderr: str


type CommandRunner = Callable[[list[str], float], CommandResult]
type ServiceRequester = Callable[[str, str, str, float], tuple[bool, str]]


def _run_command(command: list[str], timeout_seconds: float) -> CommandResult:
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
        )
    except subprocess.TimeoutExpired as error:
        raise TimeoutError(f'Gazebo command timed out: {command[0]} {command[1]}') from error
    except FileNotFoundError as error:
        raise RuntimeError(
            'Gazebo CLI was not found; source the ROS 2 Jazzy environment first'
        ) from error
    return CommandResult(result.returncode, result.stdout, result.stderr)


class _TransportServiceRequester:
    """Long-lived Gazebo Transport client backed by the official Python bindings."""

    _REQUEST_TYPES = {
        'gz.msgs.WorldControl': ('gz.msgs10.world_control_pb2', 'WorldControl'),
        'gz.msgs.Pose': ('gz.msgs10.pose_pb2', 'Pose'),
        'gz.msgs.Entity': ('gz.msgs10.entity_pb2', 'Entity'),
        'gz.msgs.EntityFactory': ('gz.msgs10.entity_factory_pb2', 'EntityFactory'),
    }

    def __init__(self) -> None:
        transport = importlib.import_module('gz.transport13')
        boolean_module = importlib.import_module('gz.msgs10.boolean_pb2')
        self._text_format = importlib.import_module('google.protobuf.text_format')
        self._node = transport.Node()
        self._boolean_type = boolean_module.Boolean

    def __call__(
        self,
        service: str,
        request_type: str,
        request_text: str,
        timeout_seconds: float,
    ) -> tuple[bool, str]:
        try:
            module_name, class_name = self._REQUEST_TYPES[request_type]
        except KeyError:
            return False, f'unsupported Gazebo request type: {request_type}'
        request_class = getattr(importlib.import_module(module_name), class_name)
        request = request_class()
        try:
            self._text_format.Parse(request_text, request)
            executed, response = self._node.request(
                service,
                request,
                request_class,
                self._boolean_type,
                max(1, round(timeout_seconds * 1000.0)),
            )
        except Exception as error:
            return False, str(error)
        accepted = bool(executed and response.data)
        return accepted, '' if accepted else 'service call timed out or was rejected'


class _CliServiceRequester:
    """Compatibility path for systems without Gazebo's Python bindings."""

    def __init__(self, runner: CommandRunner) -> None:
        self._run = runner

    def __call__(
        self,
        service: str,
        request_type: str,
        request_text: str,
        timeout_seconds: float,
    ) -> tuple[bool, str]:
        command = [
            'gz',
            'service',
            '-s',
            service,
            '--reqtype',
            request_type,
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            str(max(1, round(timeout_seconds * 1000.0))),
            '--req',
            request_text,
        ]
        try:
            result = self._run(command, timeout_seconds + 1.0)
        except (RuntimeError, TimeoutError) as error:
            return False, str(error)
        accepted = result.returncode == 0 and 'data: true' in result.stdout.lower()
        detail = (result.stderr or result.stdout).strip() or 'request rejected'
        return accepted, '' if accepted else detail


class GazeboWorldBackend:
    """Reset entities through Gazebo Transport without leaking it into algorithms.

    Jazzy's ``turtlebot4_simulator`` exposes world control through Gazebo
    Transport services, not ROS services. A long-lived official Transport node
    avoids repeated discovery overhead; the ``gz service`` executable remains a
    compatibility fallback. Core and DQN stay simulator-independent.
    """

    def __init__(
        self,
        config: GazeboWorldConfig | None = None,
        command_runner: CommandRunner | None = None,
    ) -> None:
        self.config = config or GazeboWorldConfig()
        if command_runner is not None:
            self._request: ServiceRequester = _CliServiceRequester(command_runner)
        else:
            try:
                self._request = _TransportServiceRequester()
            except (ImportError, AttributeError):
                self._request = _CliServiceRequester(_run_command)
        self._goal = Goal2D(self.config.goal_min_x, 0.0)
        self._closed = False

    @property
    def goal(self) -> Goal2D:
        return self._goal

    def _call_service(
        self,
        suffix: str,
        request_type: str,
        request: str,
        *,
        required: bool = True,
    ) -> bool:
        timeout = self.config.service_timeout_seconds
        service = f'/world/{self.config.world_name}/{suffix}'
        detail = 'request rejected'
        for attempt in range(self.config.service_retries + 1):
            accepted, detail = self._request(service, request_type, request, timeout)
            if accepted:
                return True
            if attempt < self.config.service_retries and self.config.retry_delay_seconds:
                time.sleep(self.config.retry_delay_seconds)
        if required:
            raise RuntimeError(f'Gazebo service {service} failed: {detail}')
        return False

    def _pause(self, paused: bool) -> None:
        self._call_service(
            'control',
            'gz.msgs.WorldControl',
            f'pause: {str(paused).lower()}',
        )

    @staticmethod
    def _orientation(yaw: float) -> tuple[float, float]:
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def _set_entity_pose(self, name: str, x: float, y: float, yaw: float) -> None:
        z_value, w_value = self._orientation(yaw)
        request = (
            f'name: {json.dumps(name)} '
            f'position {{ x: {x!r} y: {y!r} z: 0.0 }} '
            f'orientation {{ x: 0.0 y: 0.0 z: {z_value!r} w: {w_value!r} }}'
        )
        self._call_service('set_pose', 'gz.msgs.Pose', request)

    def _randomize_obstacles(
        self, rng: np.random.Generator
    ) -> dict[str, tuple[float, float, float]]:
        poses: dict[str, tuple[float, float, float]] = {}
        occupied = [
            (self.config.robot_start_x, self.config.robot_start_y),
            (self._goal.x, self._goal.y),
        ]
        for name in self.config.randomizable_obstacles:
            for _ in range(self.config.obstacle_sample_attempts):
                x = float(rng.uniform(self.config.obstacle_min_x, self.config.obstacle_max_x))
                y = float(rng.uniform(self.config.obstacle_min_y, self.config.obstacle_max_y))
                if all(
                    math.hypot(x - occupied_x, y - occupied_y)
                    >= self.config.obstacle_clearance
                    for occupied_x, occupied_y in occupied
                ):
                    break
            else:
                raise RuntimeError(f'could not sample a collision-free pose for obstacle {name}')
            yaw = float(rng.uniform(-math.pi, math.pi))
            self._set_entity_pose(name, x, y, yaw)
            poses[name] = (x, y, yaw)
            occupied.append((x, y))
        return poses

    def _delete_goal_marker(self, *, required: bool = False) -> bool:
        request = f'name: {json.dumps(self.config.goal_entity_name)} type: MODEL'
        return self._call_service('remove', 'gz.msgs.Entity', request, required=required)

    def _replace_goal_marker(self) -> None:
        self._delete_goal_marker()
        request = (
            f'name: {json.dumps(self.config.goal_entity_name)} '
            'allow_renaming: false '
            'pose { '
            f'position {{ x: {self._goal.x!r} y: {self._goal.y!r} '
            f'z: {self.config.goal_marker_z!r} }} '
            'orientation { w: 1.0 } } '
            f'sdf: {json.dumps(goal_marker_sdf())}'
        )
        self._call_service('create', 'gz.msgs.EntityFactory', request)

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, object] | None = None,
    ) -> dict[str, object]:
        if self._closed:
            raise RuntimeError('Gazebo world backend is closed')
        options = options or {}
        rng = np.random.default_rng(seed)
        if 'goal' in options:
            goal = options['goal']
            if not isinstance(goal, (tuple, list)) or len(goal) != 2:
                raise ValueError('goal must contain x and y')
            self._goal = Goal2D(float(goal[0]), float(goal[1]))
        else:
            self._goal = Goal2D(
                float(rng.uniform(self.config.goal_min_x, self.config.goal_max_x)),
                float(rng.uniform(self.config.goal_min_y, self.config.goal_max_y)),
            )
        self._pause(True)
        obstacle_poses: dict[str, tuple[float, float, float]] = {}
        try:
            self._set_entity_pose(
                self.config.robot_entity_name,
                self.config.robot_start_x,
                self.config.robot_start_y,
                self.config.robot_start_yaw,
            )
            if bool(options.get('randomize_obstacles', self.config.randomize_obstacles)):
                obstacle_poses = self._randomize_obstacles(rng)
            self._replace_goal_marker()
        finally:
            self._pause(False)
        if self.config.settle_seconds:
            time.sleep(self.config.settle_seconds)
        info: dict[str, object] = {'seed': seed, 'backend': 'gazebo_harmonic'}
        if obstacle_poses:
            info['obstacle_poses'] = obstacle_poses
        return info

    def set_goal(self, x: float, y: float) -> None:
        if self._closed:
            raise RuntimeError('Gazebo world backend is closed')
        self._goal = Goal2D(float(x), float(y))
        self._replace_goal_marker()

    def close(self) -> None:
        if self._closed:
            return
        self._delete_goal_marker()
        self._closed = True

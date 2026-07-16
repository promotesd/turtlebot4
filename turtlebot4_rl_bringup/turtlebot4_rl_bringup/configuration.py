"""Validated YAML-to-core configuration assembly."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from turtlebot4_rl_core.action import DiscreteVelocityAction, DiscreteVelocityConfig
from turtlebot4_rl_core.observation import LidarGoalObservation, LidarGoalObservationConfig
from turtlebot4_rl_core.reward import NavigationReward, NavigationRewardConfig
from turtlebot4_rl_core.task import PointGoalNavigationTask
from turtlebot4_rl_core.termination import NavigationTermination, NavigationTerminationConfig
import yaml


@dataclass(frozen=True)
class BackendSelection:
    """Names used to select a registered robot/world backend pair."""

    robot: str
    world: str


@dataclass(frozen=True)
class FrameworkConfiguration:
    """Fully validated environment and adapter configuration."""

    backend: BackendSelection
    task: PointGoalNavigationTask
    ros: dict[str, Any]
    gazebo: dict[str, Any]


def _mapping(value: object, label: str) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise ValueError(f'{label} must be a string-keyed mapping')
    return value


def load_configuration(path: str | Path) -> FrameworkConfiguration:
    """Load YAML and construct validated core component objects."""
    with Path(path).open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    root = _mapping(document, 'configuration')
    backend_data = _mapping(root.get('backend'), 'backend')
    robot_backend = backend_data.get('robot')
    world_backend = backend_data.get('world')
    if not isinstance(robot_backend, str) or not isinstance(world_backend, str):
        raise ValueError('backend.robot and backend.world must be strings')

    observation_data = _mapping(root.get('observation'), 'observation')
    action_data = _mapping(root.get('action'), 'action')
    termination_data = _mapping(root.get('termination'), 'termination')
    reward_data = _mapping(root.get('reward'), 'reward')
    if action_data.get('type', 'discrete_velocity') != 'discrete_velocity':
        raise ValueError('only discrete_velocity actions are currently supported')
    raw_commands = action_data.get('commands')
    if not isinstance(raw_commands, list) or not raw_commands:
        raise ValueError('action.commands must be a non-empty list')
    commands: list[tuple[float, float]] = []
    for index, item in enumerate(raw_commands):
        command = _mapping(item, f'action.commands[{index}]')
        try:
            commands.append((float(command['linear_x']), float(command['angular_z'])))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f'action.commands[{index}] requires numeric linear_x and angular_z'
            ) from error

    task = PointGoalNavigationTask(
        observation=LidarGoalObservation(LidarGoalObservationConfig(**observation_data)),
        action=DiscreteVelocityAction(
            DiscreteVelocityConfig(
                duration_seconds=float(action_data.get('duration_seconds', 0.2)),
                commands=tuple(commands),
            )
        ),
        reward=NavigationReward(NavigationRewardConfig(**reward_data)),
        termination=NavigationTermination(NavigationTerminationConfig(**termination_data)),
    )
    return FrameworkConfiguration(
        BackendSelection(robot_backend, world_backend),
        task,
        _mapping(root.get('ros'), 'ros'),
        _mapping(root.get('gazebo'), 'gazebo'),
    )

import xml.etree.ElementTree as ET

import pytest

from turtlebot4_rl_gz.backend import CommandResult, GazeboWorldBackend
from turtlebot4_rl_gz.config import GazeboWorldConfig
from turtlebot4_rl_gz.model import goal_marker_sdf


def test_goal_marker_is_valid_static_sdf() -> None:
    root = ET.fromstring(goal_marker_sdf())
    assert root.tag == 'sdf'
    assert root.findtext('./model/static') == 'true'
    assert root.find('./model/link/visual/geometry/cylinder') is not None


def test_entity_names_and_goal_bounds_are_validated() -> None:
    with pytest.raises(ValueError, match='unsupported characters'):
        GazeboWorldConfig(world_name='../unsafe')
    with pytest.raises(ValueError, match='goal bounds'):
        GazeboWorldConfig(goal_min_x=2.0, goal_max_x=1.0)


def test_reset_uses_native_gazebo_transport_services() -> None:
    commands: list[list[str]] = []

    def runner(command: list[str], timeout_seconds: float) -> CommandResult:
        assert timeout_seconds > 0
        commands.append(command)
        return CommandResult(0, 'data: true\n', '')

    backend = GazeboWorldBackend(
        GazeboWorldConfig(settle_seconds=0.0, retry_delay_seconds=0.0),
        command_runner=runner,
    )
    info = backend.reset(seed=7)
    assert info == {'seed': 7, 'backend': 'gazebo_harmonic'}
    assert [command[3] for command in commands] == [
        '/world/warehouse/control',
        '/world/warehouse/set_pose',
        '/world/warehouse/remove',
        '/world/warehouse/create',
        '/world/warehouse/control',
    ]
    assert all(command[:2] == ['gz', 'service'] for command in commands)
    assert backend.goal.x >= backend.config.goal_min_x
    assert backend.goal.x <= backend.config.goal_max_x


def test_rejected_gazebo_request_has_explicit_error() -> None:
    def runner(command: list[str], timeout_seconds: float) -> CommandResult:
        del command, timeout_seconds
        return CommandResult(0, 'data: false\n', '')

    backend = GazeboWorldBackend(
        GazeboWorldConfig(settle_seconds=0.0, retry_delay_seconds=0.0),
        command_runner=runner,
    )
    with pytest.raises(RuntimeError, match='Gazebo service .* failed'):
        backend.reset(seed=1)

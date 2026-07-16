from pathlib import Path

import pytest

from turtlebot4_rl_bringup.configuration import load_configuration
from turtlebot4_rl_bringup.factory import BackendRegistry, default_registry


CONFIGURATION = Path(__file__).parents[1] / 'config' / 'navigation.yaml'


def test_configuration_builds_parameterized_task() -> None:
    configuration = load_configuration(CONFIGURATION)
    assert configuration.backend.robot == 'ros2'
    assert configuration.backend.world == 'gazebo_harmonic'
    assert configuration.task.observation.size == 26
    assert configuration.task.action.size == 5
    assert configuration.task.termination.config.max_steps == 800


def test_registry_rejects_duplicates_and_unknown_pairs() -> None:
    configuration = load_configuration(CONFIGURATION)
    registry = BackendRegistry()
    registry.register('mock', 'mock', lambda _: object())
    with pytest.raises(ValueError, match='already registered'):
        registry.register('mock', 'mock', lambda _: object())
    with pytest.raises(ValueError, match='unknown backend pair'):
        registry.create(configuration)


def test_default_registry_constructs_mock_without_ros(tmp_path: Path) -> None:
    content = CONFIGURATION.read_text(encoding='utf-8').replace(
        'robot: ros2\n  world: gazebo_harmonic', 'robot: mock\n  world: mock'
    )
    path = tmp_path / 'mock.yaml'
    path.write_text(content, encoding='utf-8')
    environment = default_registry().create(load_configuration(path))
    observation, info = environment.reset(seed=42)
    assert observation.shape == (26,)
    assert info['backend'] == 'mock'
    environment.close()

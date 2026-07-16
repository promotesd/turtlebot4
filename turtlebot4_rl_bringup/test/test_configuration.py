from pathlib import Path

import pytest

from turtlebot4_rl_bringup.configuration import load_configuration
from turtlebot4_rl_bringup.factory import BackendRegistry, default_registry
from turtlebot4_rl_bringup.reporting import summarize
from turtlebot4_dqn.trainer import EpisodeMetrics


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


def test_scientific_report_contains_provenance_and_per_seed_metrics() -> None:
    metrics = [
        EpisodeMetrics(0, 11, 3.0, 2, True, False, False, 0.25, 1.5, 2.0, 1.0, 0.4),
        EpisodeMetrics(1, 23, -1.0, 4, False, True, False, None, 2.5, 3.0, 0.0, 0.6),
    ]
    report = summarize(
        metrics,
        'train',
        run_duration_seconds=5.0,
        provenance={'code_revision': 'abc123', 'environment_version': 'test/1'},
    )
    assert report['mean_steps'] == pytest.approx(3.0)
    assert report['mean_time_to_goal_seconds'] == pytest.approx(1.5)
    assert report['mean_training_loss'] == pytest.approx(0.25)
    assert report['train_duration_seconds'] == pytest.approx(5.0)
    assert report['code_revision'] == 'abc123'
    assert report['seeds'] == [11, 23]
    per_episode = report['per_episode']
    assert isinstance(per_episode, list)
    assert per_episode[1]['collision'] is True
    with pytest.raises(ValueError, match='must not be empty'):
        summarize([], 'train')

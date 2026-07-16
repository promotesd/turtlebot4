import numpy as np
from turtlebot4_rl_core.task import PointGoalNavigationTask
from turtlebot4_rl_core.termination import NavigationTermination, NavigationTerminationConfig

from turtlebot4_rl_mock import create_mock_environment


def rollout(seed: int) -> tuple[np.ndarray, ...]:
    env, _ = create_mock_environment()
    states = [env.reset(seed=seed, options={'randomize_goal': True})[0]]
    for action in (2, 1, 3, 2):
        states.append(env.step(action)[0])
    env.close()
    return tuple(states)


def test_same_seed_produces_same_trajectory() -> None:
    first, second = rollout(42), rollout(42)
    assert all(np.array_equal(left, right) for left, right in zip(first, second, strict=True))


def test_sensor_failure_is_truncation_and_stops_robot() -> None:
    env, simulation = create_mock_environment()
    env.reset(seed=1)
    simulation.inject_sensor_failure('stale_scan')
    _, _, terminated, truncated, info = env.step(2)
    assert not terminated and truncated
    assert info['termination_reason'] == 'stale_scan'
    assert simulation.stopped


def test_goal_and_timeout_have_distinct_end_states() -> None:
    task = PointGoalNavigationTask(
        termination=NavigationTermination(NavigationTerminationConfig(max_steps=1))
    )
    env, _ = create_mock_environment(task)
    env.reset(options={'goal': (2.0, 0.0)})
    assert env.step(2)[2:4] == (False, True)
    env.reset(options={'goal': (0.1, 0.0)})
    assert env.step(2)[2:4] == (True, False)


def test_one_thousand_resets_do_not_leak_episode_state() -> None:
    task = PointGoalNavigationTask(
        termination=NavigationTermination(NavigationTerminationConfig(max_steps=1))
    )
    env, simulation = create_mock_environment(task)
    for seed in range(1000):
        _, info = env.reset(seed=seed, options={'randomize_goal': True})
        assert env.step_count == 0
        assert simulation.clock_seconds == 0.0
        assert info['seed'] == seed
        _, _, terminated, truncated, _ = env.step(2)
        assert not terminated and truncated
        assert env.step_count == 1
    env.close()


def test_close_is_idempotent_and_stops_robot() -> None:
    env, simulation = create_mock_environment()
    env.reset()
    env.step(2)
    env.close()
    env.close()
    assert simulation.stopped

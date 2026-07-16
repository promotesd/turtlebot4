from pathlib import Path

import numpy as np
import pytest
from turtlebot4_dqn.agent import DQNAgent, DQNConfig
from turtlebot4_dqn.checkpoint import load_checkpoint, save_checkpoint
from turtlebot4_dqn.configuration import load_dqn_configuration
from turtlebot4_dqn.evaluator import evaluate
from turtlebot4_dqn.exploration import LinearEpsilonSchedule
from turtlebot4_dqn.replay_buffer import ReplayBuffer, Transition
from turtlebot4_dqn.trainer import train

CONFIGURATION = Path(__file__).parents[1] / 'turtlebot4_dqn' / 'config' / 'dqn.yaml'


def transition(value: float, *, terminated: bool = False) -> Transition:
    state = np.full(4, value, dtype=np.float32)
    return Transition(state, 1, value, state + 1, terminated, False)


def test_replay_buffer_is_bounded_and_seeded() -> None:
    first, second = ReplayBuffer(3, seed=7), ReplayBuffer(3, seed=7)
    for index in range(5):
        first.append(transition(float(index)))
        second.append(transition(float(index)))
    assert len(first) == 3
    assert [item.reward for item in first.sample(2)] == [item.reward for item in second.sample(2)]


def test_epsilon_decay_is_bounded() -> None:
    schedule = LinearEpsilonSchedule(initial=1.0, final=0.1, decay_steps=10)
    assert schedule.value(0) == 1.0
    assert schedule.value(5) == pytest.approx(0.55)
    assert schedule.value(20) == 0.1


def test_dqn_yaml_is_fully_validated() -> None:
    configuration = load_dqn_configuration(CONFIGURATION)
    assert configuration.agent.gamma == pytest.approx(0.99)
    assert configuration.exploration.final == pytest.approx(0.05)
    assert configuration.evaluation_seeds == (11, 23, 47, 89, 131)


def test_agent_learns_and_target_network_updates() -> None:
    config = DQNConfig(
        batch_size=2,
        replay_capacity=10,
        warmup_steps=2,
        target_update_interval=1,
        hidden_size=8,
    )
    agent = DQNAgent(4, 3, config, seed=1)
    agent.remember(transition(0.0))
    agent.remember(transition(1.0, terminated=True))
    loss = agent.learn()
    assert loss is not None and np.isfinite(loss)
    assert agent.training_steps == 1
    for online, target in zip(
        agent.online_network.get_weights(), agent.target_network.get_weights(), strict=True
    ):
        assert np.array_equal(online, target)


def test_checkpoint_round_trip(tmp_path: Path) -> None:
    original = DQNAgent(4, 3, DQNConfig(hidden_size=8), seed=2)
    original.environment_steps = 24
    original.training_steps = 42
    path = save_checkpoint(original, tmp_path / 'agent.npz')
    restored = DQNAgent(4, 3, DQNConfig(hidden_size=8), seed=99)
    load_checkpoint(restored, path)
    assert restored.environment_steps == 24
    assert restored.training_steps == 42
    state = np.arange(4, dtype=np.float32)
    assert np.array_equal(
        original.online_network.predict(state), restored.online_network.predict(state)
    )


class TwoStepEnvironment:
    def __init__(self) -> None:
        self.step_count = 0

    def reset(self, *, seed=None, options=None):
        self.step_count = 0
        return np.zeros(4, dtype=np.float32), {
            'goal_distance': 2.0,
            'robot_pose': (0.0, 0.0, 0.0),
        }

    def step(self, action):
        self.step_count += 1
        terminated = self.step_count == 2
        return (
            np.full(4, self.step_count, dtype=np.float32),
            1.0,
            terminated,
            False,
            {
                'step': self.step_count,
                'robot_pose': (float(self.step_count), 0.0, 0.0),
                'termination_reason': 'goal_reached' if terminated else None,
            },
        )


def test_train_and_evaluate_record_scientific_metrics() -> None:
    environment = TwoStepEnvironment()
    config = DQNConfig(batch_size=2, replay_capacity=10, warmup_steps=2, hidden_size=8)
    agent = DQNAgent(4, 3, config, seed=5)
    training = train(environment, agent, 1, seed=10)[0]
    assert training.path_length == pytest.approx(2.0)
    assert training.spl == pytest.approx(1.0)
    assert training.duration_seconds > 0.0
    assert training.mean_inference_ms >= 0.0
    assert agent.environment_steps == 2

    evaluation = evaluate(environment, agent, [20])[0]
    assert evaluation.success
    assert evaluation.path_length == pytest.approx(2.0)
    assert evaluation.spl == pytest.approx(1.0)
    assert agent.environment_steps == 2

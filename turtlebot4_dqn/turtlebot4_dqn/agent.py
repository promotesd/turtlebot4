"""Backend-independent DQN agent."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from turtlebot4_dqn.exploration import LinearEpsilonSchedule
from turtlebot4_dqn.network import NumpyQNetwork, QNetwork
from turtlebot4_dqn.replay_buffer import ReplayBuffer, Transition


@dataclass(frozen=True)
class DQNConfig:
    """Validated DQN hyperparameters."""

    gamma: float = 0.99
    learning_rate: float = 0.001
    batch_size: int = 64
    replay_capacity: int = 100_000
    warmup_steps: int = 1_000
    target_update_interval: int = 500
    hidden_size: int = 64

    def __post_init__(self) -> None:
        if not 0.0 <= self.gamma <= 1.0:
            raise ValueError('gamma must be in [0, 1]')
        if self.learning_rate <= 0:
            raise ValueError('learning_rate must be positive')
        if (
            min(
                self.batch_size,
                self.replay_capacity,
                self.warmup_steps,
                self.target_update_interval,
                self.hidden_size,
            )
            <= 0
        ):
            raise ValueError('integer DQN hyperparameters must be positive')


class DQNAgent:
    """DQN policy and update rule with an explicit target network."""

    def __init__(
        self,
        observation_size: int,
        action_size: int,
        config: DQNConfig | None = None,
        *,
        seed: int = 0,
        online_network: QNetwork | None = None,
        target_network: QNetwork | None = None,
        exploration: LinearEpsilonSchedule | None = None,
    ) -> None:
        self.config = config or DQNConfig()
        self.action_size = action_size
        self.online_network = online_network or NumpyQNetwork(
            observation_size, action_size, self.config.hidden_size, seed
        )
        self.target_network = target_network or NumpyQNetwork(
            observation_size, action_size, self.config.hidden_size, seed + 1
        )
        self.target_network.set_weights(self.online_network.get_weights())
        self.replay = ReplayBuffer(self.config.replay_capacity, seed)
        self.exploration = exploration or LinearEpsilonSchedule()
        self._rng = np.random.default_rng(seed)
        self.environment_steps = 0
        self.training_steps = 0

    def select_action(self, observation: npt.NDArray[np.float32], *, training: bool = True) -> int:
        epsilon = self.exploration.value(self.environment_steps) if training else 0.0
        if training:
            self.environment_steps += 1
            if self._rng.random() < epsilon:
                return int(self._rng.integers(self.action_size))
        q_values = self.online_network.predict(observation)[0]
        return int(np.argmax(q_values))

    def remember(self, transition: Transition) -> None:
        self.replay.append(transition)

    def learn(self) -> float | None:
        minimum = max(self.config.batch_size, self.config.warmup_steps)
        if len(self.replay) < minimum:
            return None
        batch = self.replay.sample(self.config.batch_size)
        observations = np.stack([item.observation for item in batch])
        next_observations = np.stack([item.next_observation for item in batch])
        targets = self.online_network.predict(observations).copy()
        next_values = self.target_network.predict(next_observations).max(axis=1)
        for index, item in enumerate(batch):
            # Gymnasium time limits are not MDP terminal states, so bootstrap through truncation.
            bootstrap = 0.0 if item.terminated else self.config.gamma * next_values[index]
            targets[index, item.action] = item.reward + bootstrap
        loss = self.online_network.train_batch(observations, targets, self.config.learning_rate)
        self.training_steps += 1
        if self.training_steps % self.config.target_update_interval == 0:
            self.target_network.set_weights(self.online_network.get_weights())
        return loss

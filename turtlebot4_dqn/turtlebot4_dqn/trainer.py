"""Training loop that depends only on the Gymnasium-style environment API."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any, Protocol

import numpy as np
import numpy.typing as npt

from turtlebot4_dqn.agent import DQNAgent
from turtlebot4_dqn.replay_buffer import Transition


class Environment(Protocol):
    """Environment surface required by trainer/evaluator."""

    def reset(
        self, *, seed: int | None = None, options: dict[str, object] | None = None
    ) -> tuple[npt.NDArray[np.float32], dict[str, Any]]: ...

    def step(
        self, action: int
    ) -> tuple[npt.NDArray[np.float32], float, bool, bool, dict[str, Any]]: ...


@dataclass(frozen=True)
class EpisodeMetrics:
    """Stable per-episode metrics for comparisons across seeds."""

    episode: int
    seed: int
    reward: float
    steps: int
    success: bool
    collision: bool
    truncated: bool
    mean_loss: float | None
    duration_seconds: float
    path_length: float
    spl: float
    mean_inference_ms: float


def _pose_xy(info: dict[str, Any]) -> tuple[float, float]:
    pose = info.get('robot_pose')
    if not isinstance(pose, (tuple, list)) or len(pose) < 2:
        raise ValueError('environment info must contain robot_pose=(x, y, yaw)')
    return float(pose[0]), float(pose[1])


def _path_increment(previous: tuple[float, float], current: tuple[float, float]) -> float:
    return math.hypot(current[0] - previous[0], current[1] - previous[1])


def _spl(success: bool, optimal_distance: float, path_length: float) -> float:
    if not success:
        return 0.0
    return optimal_distance / max(optimal_distance, path_length, 1e-9)


def train(
    environment: Environment,
    agent: DQNAgent,
    episodes: int,
    *,
    seed: int = 0,
) -> list[EpisodeMetrics]:
    """Train for a fixed, reproducible set of episode seeds."""
    if episodes <= 0:
        raise ValueError('episodes must be positive')
    results: list[EpisodeMetrics] = []
    for episode in range(episodes):
        episode_seed = seed + episode
        episode_started = time.perf_counter()
        observation, reset_info = environment.reset(seed=episode_seed)
        optimal_distance = float(reset_info['goal_distance'])
        previous_pose = _pose_xy(reset_info)
        path_length = 0.0
        total_reward = 0.0
        losses: list[float] = []
        inference_seconds: list[float] = []
        while True:
            inference_started = time.perf_counter()
            action = agent.select_action(observation, training=True)
            inference_seconds.append(time.perf_counter() - inference_started)
            next_observation, reward, terminated, truncated, info = environment.step(action)
            current_pose = _pose_xy(info)
            path_length += _path_increment(previous_pose, current_pose)
            previous_pose = current_pose
            agent.remember(
                Transition(
                    observation,
                    action,
                    reward,
                    next_observation,
                    terminated,
                    truncated,
                )
            )
            loss = agent.learn()
            if loss is not None:
                losses.append(loss)
            total_reward += reward
            observation = next_observation
            if terminated or truncated:
                reason = info.get('termination_reason')
                success = reason == 'goal_reached'
                results.append(
                    EpisodeMetrics(
                        episode,
                        episode_seed,
                        total_reward,
                        int(info['step']),
                        success,
                        reason == 'collision',
                        truncated,
                        float(np.mean(losses)) if losses else None,
                        time.perf_counter() - episode_started,
                        path_length,
                        _spl(success, optimal_distance, path_length),
                        float(np.mean(inference_seconds) * 1000.0),
                    )
                )
                break
    return results

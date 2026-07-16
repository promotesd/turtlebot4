"""Evaluation without exploration or learning side effects."""

from __future__ import annotations

import time

import numpy as np

from turtlebot4_dqn.agent import DQNAgent
from turtlebot4_dqn.trainer import (
    _path_increment,
    _pose_xy,
    _spl,
    Environment,
    EpisodeMetrics,
)


def evaluate(
    environment: Environment,
    agent: DQNAgent,
    seeds: list[int],
) -> list[EpisodeMetrics]:
    """Evaluate a frozen greedy policy over explicit seeds."""
    if not seeds:
        raise ValueError('at least one evaluation seed is required')
    results: list[EpisodeMetrics] = []
    for episode, seed in enumerate(seeds):
        episode_started = time.perf_counter()
        observation, reset_info = environment.reset(seed=seed)
        optimal_distance = float(reset_info['goal_distance'])
        previous_pose = _pose_xy(reset_info)
        path_length = 0.0
        total_reward = 0.0
        inference_seconds: list[float] = []
        while True:
            inference_started = time.perf_counter()
            action = agent.select_action(observation, training=False)
            inference_seconds.append(time.perf_counter() - inference_started)
            observation, reward, terminated, truncated, info = environment.step(action)
            current_pose = _pose_xy(info)
            path_length += _path_increment(previous_pose, current_pose)
            previous_pose = current_pose
            total_reward += reward
            if terminated or truncated:
                reason = info.get('termination_reason')
                success = reason == 'goal_reached'
                results.append(
                    EpisodeMetrics(
                        episode,
                        seed,
                        total_reward,
                        int(info['step']),
                        success,
                        reason == 'collision',
                        truncated,
                        None,
                        time.perf_counter() - episode_started,
                        path_length,
                        _spl(success, optimal_distance, path_length),
                        float(np.mean(inference_seconds) * 1000.0),
                    )
                )
                break
    return results

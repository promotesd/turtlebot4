"""Gymnasium-style TurtleBot 4 environment orchestration."""

from __future__ import annotations

from typing import Any

import numpy as np

from turtlebot4_rl_core.backend import RobotBackend, WorldBackend
from turtlebot4_rl_core.geometry import goal_distance
from turtlebot4_rl_core.spaces import Box, Discrete
from turtlebot4_rl_core.task import PointGoalNavigationTask
from turtlebot4_rl_core.types import Observation


class TurtleBot4Env:
    """Backend-independent environment with the Gymnasium reset/step API."""

    metadata: dict[str, list[str]] = {'render_modes': []}

    def __init__(
        self,
        robot: RobotBackend,
        world: WorldBackend,
        task: PointGoalNavigationTask | None = None,
    ) -> None:
        self.robot = robot
        self.world = world
        self.task = task or PointGoalNavigationTask()
        self.action_space = Discrete(self.task.action.size)
        lidar_max = self.task.observation.config.lidar_max_range
        lows = np.zeros(self.task.observation.size, dtype=np.float32)
        highs = np.full(self.task.observation.size, lidar_max, dtype=np.float32)
        cursor = self.task.observation.config.lidar_bins
        if self.task.observation.config.include_goal_distance:
            cursor += 1
        if self.task.observation.config.include_goal_angle:
            lows[cursor] = -np.pi
            highs[cursor] = np.pi
        self.observation_space = Box(lows, highs)
        self._step_count = 0
        self._active = False
        self._closed = False

    @property
    def step_count(self) -> int:
        return self._step_count

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, object] | None = None,
    ) -> tuple[Observation, dict[str, Any]]:
        if self._closed:
            raise RuntimeError('environment is closed')
        self.robot.stop()
        world_info = self.world.reset(seed=seed, options=options)
        self.action_space.seed(seed)
        if not self.robot.is_ready():
            raise RuntimeError('robot backend is not ready after reset')
        sample = self.robot.read_observation()
        if not sample.healthy:
            raise RuntimeError(f'invalid initial sensor sample: {sample.status}')
        distance = goal_distance(sample.pose, self.world.goal)
        self.task.reward.reset(distance)
        self._step_count = 0
        self._active = True
        observation = self.task.observation.build(sample, self.world.goal)
        info: dict[str, Any] = {
            **world_info,
            'goal': (self.world.goal.x, self.world.goal.y),
            'goal_distance': distance,
            'robot_pose': (sample.pose.x, sample.pose.y, sample.pose.yaw),
            'step': 0,
        }
        return observation, info

    def step(self, action: int) -> tuple[Observation, float, bool, bool, dict[str, Any]]:
        if self._closed:
            raise RuntimeError('environment is closed')
        if not self._active:
            raise RuntimeError('reset must be called before step')
        command = self.task.action.command(action)
        try:
            self.robot.execute_action(command)
            sample = self.robot.read_observation()
        except Exception:
            self.robot.stop()
            self._active = False
            raise
        self._step_count += 1
        distance = goal_distance(sample.pose, self.world.goal)
        terminated, truncated, reason = self.task.termination.classify(
            sample, distance, self._step_count
        )
        reached_goal = reason == 'goal_reached'
        collided = reason == 'collision'
        reward, reward_components = self.task.reward.calculate(
            distance, reached_goal=reached_goal, collided=collided
        )
        observation = self.task.observation.build(sample, self.world.goal)
        if terminated or truncated:
            self.robot.stop()
            self._active = False
        info = {
            'goal_distance': distance,
            'robot_pose': (sample.pose.x, sample.pose.y, sample.pose.yaw),
            'step': self._step_count,
            'termination_reason': reason,
            'reward_components': reward_components,
            'sensor_status': sample.status,
        }
        return observation, reward, terminated, truncated, info

    def close(self) -> None:
        if self._closed:
            return
        self.robot.stop()
        self.robot.close()
        self.world.close()
        self._active = False
        self._closed = True

    def __enter__(self) -> TurtleBot4Env:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

"""Lidar and relative-goal observation features."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from turtlebot4_rl_core.geometry import goal_distance, goal_heading_error
from turtlebot4_rl_core.types import Goal2D, Observation, RobotObservation


@dataclass(frozen=True)
class LidarGoalObservationConfig:
    """Validated feature-extraction parameters."""

    lidar_bins: int = 24
    lidar_min_range: float = 0.12
    lidar_max_range: float = 10.0
    include_goal_distance: bool = True
    include_goal_angle: bool = True

    def __post_init__(self) -> None:
        if self.lidar_bins <= 0:
            raise ValueError('lidar_bins must be positive')
        if self.lidar_min_range < 0 or self.lidar_max_range <= self.lidar_min_range:
            raise ValueError('invalid lidar range bounds')


class LidarGoalObservation:
    """Min-pools lidar sectors and appends bounded goal features."""

    def __init__(self, config: LidarGoalObservationConfig | None = None) -> None:
        self.config = config or LidarGoalObservationConfig()

    @property
    def size(self) -> int:
        return (
            self.config.lidar_bins
            + int(self.config.include_goal_distance)
            + int(self.config.include_goal_angle)
        )

    def _downsample(self, ranges: npt.NDArray[np.float32]) -> npt.NDArray[np.float32]:
        values = np.asarray(ranges, dtype=np.float32).reshape(-1)
        if values.size == 0:
            raise ValueError('laser ranges must not be empty')
        values = np.nan_to_num(
            values,
            nan=self.config.lidar_max_range,
            posinf=self.config.lidar_max_range,
            neginf=self.config.lidar_min_range,
        )
        values = np.clip(values, self.config.lidar_min_range, self.config.lidar_max_range)
        boundaries = np.linspace(0, values.size, self.config.lidar_bins + 1, dtype=int)
        pooled = np.empty(self.config.lidar_bins, dtype=np.float32)
        for index in range(self.config.lidar_bins):
            start, stop = boundaries[index], boundaries[index + 1]
            if start == stop:
                pooled[index] = values[min(start, values.size - 1)]
            else:
                pooled[index] = float(np.min(values[start:stop]))
        return pooled

    def build(self, sample: RobotObservation, goal: Goal2D) -> Observation:
        features = [self._downsample(sample.ranges)]
        if self.config.include_goal_distance:
            features.append(
                np.asarray(
                    [min(goal_distance(sample.pose, goal), self.config.lidar_max_range)],
                    dtype=np.float32,
                )
            )
        if self.config.include_goal_angle:
            features.append(np.asarray([goal_heading_error(sample.pose, goal)], dtype=np.float32))
        return np.concatenate(features, dtype=np.float32)

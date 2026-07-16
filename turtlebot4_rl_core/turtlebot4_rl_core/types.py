"""Shared immutable value types used at the core boundary."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

type Observation = npt.NDArray[np.float32]


@dataclass(frozen=True)
class Pose2D:
    """Planar robot pose in metres and radians."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Goal2D:
    """Planar navigation target in metres."""

    x: float
    y: float


@dataclass(frozen=True)
class VelocityCommand:
    """Robot-relative planar velocity command."""

    linear_x: float
    angular_z: float
    duration_seconds: float = 0.2


@dataclass(frozen=True)
class RobotObservation:
    """Raw backend sample before task-specific feature extraction."""

    ranges: npt.NDArray[np.float32]
    pose: Pose2D
    stamp_seconds: float
    healthy: bool = True
    status: str = 'ok'

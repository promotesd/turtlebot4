"""Pure planar geometry helpers."""

import math

from .types import Goal2D, Pose2D


def normalize_angle(angle: float) -> float:
    """Normalize an angle into [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def goal_distance(pose: Pose2D, goal: Goal2D) -> float:
    """Return Euclidean distance between a robot and target."""
    return math.hypot(goal.x - pose.x, goal.y - pose.y)


def goal_heading_error(pose: Pose2D, goal: Goal2D) -> float:
    """Return target bearing relative to robot heading."""
    bearing = math.atan2(goal.y - pose.y, goal.x - pose.x)
    return normalize_angle(bearing - pose.yaw)

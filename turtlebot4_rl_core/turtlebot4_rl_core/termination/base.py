"""Termination policy protocol."""

from typing import Protocol

from turtlebot4_rl_core.types import RobotObservation


class TerminationPolicy(Protocol):
    """Classifies MDP termination separately from external truncation."""

    def classify(
        self, sample: RobotObservation, distance: float, step_count: int
    ) -> tuple[bool, bool, str | None]: ...

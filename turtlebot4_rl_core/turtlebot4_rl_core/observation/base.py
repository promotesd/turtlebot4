"""Observation builder protocol."""

from typing import Protocol

from turtlebot4_rl_core.types import Goal2D, Observation, RobotObservation


class ObservationBuilder(Protocol):
    """Converts backend sensor samples into agent features."""

    @property
    def size(self) -> int: ...

    def build(self, sample: RobotObservation, goal: Goal2D) -> Observation: ...

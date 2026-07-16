"""Reward function protocol."""

from typing import Protocol


class RewardFunction(Protocol):
    """Produces a scalar reward and named components."""

    def reset(self, initial_distance: float) -> None: ...

    def calculate(
        self, distance: float, *, reached_goal: bool, collided: bool
    ) -> tuple[float, dict[str, float]]: ...

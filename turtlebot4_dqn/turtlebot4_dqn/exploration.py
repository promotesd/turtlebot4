"""Exploration schedules."""

from dataclasses import dataclass


@dataclass(frozen=True)
class LinearEpsilonSchedule:
    """Linearly decay epsilon over a configured number of steps."""

    initial: float = 1.0
    final: float = 0.05
    decay_steps: int = 100_000

    def __post_init__(self) -> None:
        if not 0.0 <= self.final <= self.initial <= 1.0:
            raise ValueError('epsilon values must satisfy 0 <= final <= initial <= 1')
        if self.decay_steps <= 0:
            raise ValueError('decay_steps must be positive')

    def value(self, step: int) -> float:
        if step >= self.decay_steps:
            return self.final
        progress = min(max(step, 0) / self.decay_steps, 1.0)
        return self.initial + progress * (self.final - self.initial)

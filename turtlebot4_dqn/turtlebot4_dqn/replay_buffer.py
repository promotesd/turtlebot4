"""Bounded, seedable replay memory."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt


@dataclass(frozen=True)
class Transition:
    """One environment transition with separate terminal flags."""

    observation: npt.NDArray[np.float32]
    action: int
    reward: float
    next_observation: npt.NDArray[np.float32]
    terminated: bool
    truncated: bool


class ReplayBuffer:
    """Fixed-capacity replay buffer with reproducible sampling."""

    def __init__(self, capacity: int, seed: int = 0) -> None:
        if capacity <= 0:
            raise ValueError('capacity must be positive')
        self._items: deque[Transition] = deque(maxlen=capacity)
        self._rng = np.random.default_rng(seed)

    @property
    def capacity(self) -> int:
        return int(self._items.maxlen or 0)

    def __len__(self) -> int:
        return len(self._items)

    def append(self, transition: Transition) -> None:
        self._items.append(
            Transition(
                np.asarray(transition.observation, dtype=np.float32).copy(),
                int(transition.action),
                float(transition.reward),
                np.asarray(transition.next_observation, dtype=np.float32).copy(),
                bool(transition.terminated),
                bool(transition.truncated),
            )
        )

    def sample(self, batch_size: int) -> list[Transition]:
        if batch_size <= 0:
            raise ValueError('batch_size must be positive')
        if batch_size > len(self._items):
            raise ValueError('not enough transitions to sample')
        indices = self._rng.choice(len(self._items), size=batch_size, replace=False)
        return [self._items[int(index)] for index in indices]

"""Small dependency-free spaces compatible with the Gymnasium calling style."""

from __future__ import annotations

import numpy as np
import numpy.typing as npt


class Discrete:
    """Discrete action space with deterministic seeding."""

    def __init__(self, n: int, seed: int | None = None) -> None:
        if n <= 0:
            raise ValueError('n must be positive')
        self.n = n
        self._rng = np.random.default_rng(seed)

    def seed(self, seed: int | None = None) -> list[int | None]:
        self._rng = np.random.default_rng(seed)
        return [seed]

    def sample(self) -> int:
        return int(self._rng.integers(self.n))

    def contains(self, value: object) -> bool:
        return isinstance(value, (int, np.integer)) and 0 <= int(value) < self.n


class Box:
    """Finite float32 box used to describe observations."""

    def __init__(self, low: npt.ArrayLike, high: npt.ArrayLike) -> None:
        self.low = np.asarray(low, dtype=np.float32)
        self.high = np.asarray(high, dtype=np.float32)
        if self.low.shape != self.high.shape:
            raise ValueError('low and high shapes must match')
        self.shape = self.low.shape
        self.dtype = np.dtype(np.float32)

    def contains(self, value: object) -> bool:
        array = np.asarray(value)
        return bool(
            array.shape == self.shape
            and np.all(np.isfinite(array))
            and np.all(array >= self.low)
            and np.all(array <= self.high)
        )

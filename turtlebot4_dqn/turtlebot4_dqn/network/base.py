"""Q-network protocol used by the DQN agent."""

from typing import Protocol

import numpy as np
import numpy.typing as npt


class QNetwork(Protocol):
    """Minimal interface needed by DQN."""

    def predict(self, observations: npt.NDArray[np.float32]) -> npt.NDArray[np.float32]: ...

    def train_batch(
        self,
        observations: npt.NDArray[np.float32],
        targets: npt.NDArray[np.float32],
        learning_rate: float,
    ) -> float: ...

    def get_weights(self) -> list[npt.NDArray[np.float32]]: ...

    def set_weights(self, weights: list[npt.NDArray[np.float32]]) -> None: ...

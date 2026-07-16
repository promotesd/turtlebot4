"""Small fully connected NumPy Q-network for a zero-framework baseline."""

from __future__ import annotations

import numpy as np
import numpy.typing as npt


class NumpyQNetwork:
    """One-hidden-layer MLP trained with mean squared error."""

    def __init__(
        self, observation_size: int, action_size: int, hidden_size: int = 64, seed: int = 0
    ) -> None:
        if min(observation_size, action_size, hidden_size) <= 0:
            raise ValueError('network dimensions must be positive')
        self.observation_size = observation_size
        self.action_size = action_size
        rng = np.random.default_rng(seed)
        scale1 = np.sqrt(2.0 / observation_size)
        scale2 = np.sqrt(2.0 / hidden_size)
        self.w1 = (
            rng.standard_normal((observation_size, hidden_size)) * scale1
        ).astype(np.float32)
        self.b1 = np.zeros(hidden_size, dtype=np.float32)
        self.w2 = (rng.standard_normal((hidden_size, action_size)) * scale2).astype(np.float32)
        self.b2 = np.zeros(action_size, dtype=np.float32)

    def _forward(
        self, observations: npt.NDArray[np.float32]
    ) -> tuple[npt.NDArray[np.float32], npt.NDArray[np.float32]]:
        values = np.asarray(observations, dtype=np.float32)
        if values.ndim == 1:
            values = values[None, :]
        if values.shape[1] != self.observation_size:
            raise ValueError('observation size does not match network input')
        hidden = np.maximum(values @ self.w1 + self.b1, 0.0)
        return hidden, hidden @ self.w2 + self.b2

    def predict(self, observations: npt.NDArray[np.float32]) -> npt.NDArray[np.float32]:
        return self._forward(observations)[1].astype(np.float32, copy=False)

    def train_batch(
        self,
        observations: npt.NDArray[np.float32],
        targets: npt.NDArray[np.float32],
        learning_rate: float,
    ) -> float:
        if learning_rate <= 0:
            raise ValueError('learning_rate must be positive')
        values = np.asarray(observations, dtype=np.float32)
        if values.ndim == 1:
            values = values[None, :]
        expected = np.asarray(targets, dtype=np.float32)
        hidden, predicted = self._forward(values)
        if expected.shape != predicted.shape:
            raise ValueError('target shape does not match network output')
        error = predicted - expected
        loss = float(np.mean(error * error))
        output_gradient = 2.0 * error / error.size
        w2_gradient = hidden.T @ output_gradient
        b2_gradient = output_gradient.sum(axis=0)
        hidden_gradient = (output_gradient @ self.w2.T) * (hidden > 0.0)
        w1_gradient = values.T @ hidden_gradient
        b1_gradient = hidden_gradient.sum(axis=0)
        self.w2 -= learning_rate * w2_gradient
        self.b2 -= learning_rate * b2_gradient
        self.w1 -= learning_rate * w1_gradient
        self.b1 -= learning_rate * b1_gradient
        return loss

    def get_weights(self) -> list[npt.NDArray[np.float32]]:
        return [value.copy() for value in (self.w1, self.b1, self.w2, self.b2)]

    def set_weights(self, weights: list[npt.NDArray[np.float32]]) -> None:
        current = (self.w1, self.b1, self.w2, self.b2)
        if len(weights) != len(current):
            raise ValueError('unexpected number of weight arrays')
        for destination, source in zip(current, weights, strict=True):
            value = np.asarray(source, dtype=np.float32)
            if value.shape != destination.shape:
                raise ValueError('weight shape mismatch')
            destination[...] = value

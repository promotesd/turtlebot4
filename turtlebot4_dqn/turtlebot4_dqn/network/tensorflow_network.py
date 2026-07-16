"""Optional TensorFlow implementation loaded only when requested."""

from __future__ import annotations

import numpy as np
import numpy.typing as npt


class TensorFlowQNetwork:
    """Keras-backed network implementing the common Q-network protocol."""

    def __init__(self, observation_size: int, action_size: int, hidden_size: int = 64) -> None:
        try:
            import tensorflow as tf
        except ImportError as error:
            raise RuntimeError(
                'TensorFlow backend requested but tensorflow is not installed'
            ) from error
        self._tf = tf
        self.model = tf.keras.Sequential(
            [
                tf.keras.layers.Input(shape=(observation_size,)),
                tf.keras.layers.Dense(hidden_size, activation='relu'),
                tf.keras.layers.Dense(hidden_size, activation='relu'),
                tf.keras.layers.Dense(action_size),
            ]
        )

    def predict(self, observations: npt.NDArray[np.float32]) -> npt.NDArray[np.float32]:
        values = np.asarray(observations, dtype=np.float32)
        if values.ndim == 1:
            values = values[None, :]
        return np.asarray(self.model(values, training=False).numpy(), dtype=np.float32)

    def train_batch(
        self,
        observations: npt.NDArray[np.float32],
        targets: npt.NDArray[np.float32],
        learning_rate: float,
    ) -> float:
        self.model.optimizer = self._tf.keras.optimizers.Adam(learning_rate)
        self.model.compile(optimizer=self.model.optimizer, loss='mse')
        return float(self.model.train_on_batch(observations, targets))

    def get_weights(self) -> list[npt.NDArray[np.float32]]:
        return [np.asarray(value, dtype=np.float32) for value in self.model.get_weights()]

    def set_weights(self, weights: list[npt.NDArray[np.float32]]) -> None:
        self.model.set_weights(weights)

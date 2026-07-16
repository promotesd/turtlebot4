"""Portable NumPy checkpoint persistence."""

from pathlib import Path
from typing import Any, cast

import numpy as np
import numpy.typing as npt

from turtlebot4_dqn.agent import DQNAgent


def save_checkpoint(agent: DQNAgent, path: str | Path) -> Path:
    """Atomically save online/target weights and training step count."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + '.tmp')
    arrays: dict[str, npt.NDArray[Any]] = {
        'environment_steps': np.asarray([agent.environment_steps], dtype=np.int64),
        'training_steps': np.asarray([agent.training_steps], dtype=np.int64)
    }
    arrays.update(
        {
            f'online_{index}': value
            for index, value in enumerate(agent.online_network.get_weights())
        }
    )
    arrays.update(
        {
            f'target_{index}': value
            for index, value in enumerate(agent.target_network.get_weights())
        }
    )
    with temporary.open('wb') as stream:
        np.savez_compressed(stream, **cast(Any, arrays))
    temporary.replace(destination)
    return destination


def load_checkpoint(agent: DQNAgent, path: str | Path) -> None:
    """Load a checkpoint after validating its weight groups."""
    with np.load(Path(path), allow_pickle=False) as data:
        online_keys = sorted(
            (key for key in data.files if key.startswith('online_')),
            key=lambda key: int(key.split('_')[1]),
        )
        target_keys = sorted(
            (key for key in data.files if key.startswith('target_')),
            key=lambda key: int(key.split('_')[1]),
        )
        if not online_keys or len(online_keys) != len(target_keys):
            raise ValueError('checkpoint does not contain complete network weights')
        agent.online_network.set_weights([data[key] for key in online_keys])
        agent.target_network.set_weights([data[key] for key in target_keys])
        agent.environment_steps = int(data['environment_steps'][0])
        agent.training_steps = int(data['training_steps'][0])

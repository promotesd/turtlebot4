"""Validated DQN YAML configuration loading."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from turtlebot4_dqn.agent import DQNConfig
from turtlebot4_dqn.exploration import LinearEpsilonSchedule
import yaml


@dataclass(frozen=True)
class LoadedDQNConfiguration:
    """Agent, exploration, and evaluation configuration."""

    agent: DQNConfig
    exploration: LinearEpsilonSchedule
    evaluation_seeds: tuple[int, ...]


def _mapping(value: object, label: str) -> dict[str, Any]:
    if not isinstance(value, dict) or not all(isinstance(key, str) for key in value):
        raise ValueError(f'{label} must be a string-keyed mapping')
    return value


def load_dqn_configuration(path: str | Path) -> LoadedDQNConfiguration:
    """Load and validate every public section of ``dqn.yaml``."""
    with Path(path).open(encoding='utf-8') as stream:
        document = _mapping(yaml.safe_load(stream), 'configuration')
    evaluation = _mapping(document.get('evaluation'), 'evaluation')
    raw_seeds = evaluation.get('seeds')
    if not isinstance(raw_seeds, list) or not raw_seeds:
        raise ValueError('evaluation.seeds must be a non-empty list')
    if any(isinstance(seed, bool) or not isinstance(seed, int) for seed in raw_seeds):
        raise ValueError('evaluation seeds must be integers')
    return LoadedDQNConfiguration(
        DQNConfig(**_mapping(document.get('dqn'), 'dqn')),
        LinearEpsilonSchedule(**_mapping(document.get('exploration'), 'exploration')),
        tuple(raw_seeds),
    )

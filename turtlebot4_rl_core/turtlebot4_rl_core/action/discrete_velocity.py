"""Configurable discrete velocity actions."""

from dataclasses import dataclass

from turtlebot4_rl_core.types import VelocityCommand


def _default_commands() -> tuple[tuple[float, float], ...]:
    return ((0.12, 0.8), (0.15, 0.4), (0.18, 0.0), (0.15, -0.4), (0.12, -0.8))


@dataclass(frozen=True)
class DiscreteVelocityConfig:
    """Discrete command set and integration duration."""

    duration_seconds: float = 0.2
    commands: tuple[tuple[float, float], ...] = _default_commands()

    def __post_init__(self) -> None:
        if self.duration_seconds <= 0:
            raise ValueError('duration_seconds must be positive')
        if not self.commands:
            raise ValueError('at least one command is required')


class DiscreteVelocityAction:
    """Translate a discrete action index into linear/angular velocity."""

    def __init__(self, config: DiscreteVelocityConfig | None = None) -> None:
        self.config = config or DiscreteVelocityConfig()

    @property
    def size(self) -> int:
        return len(self.config.commands)

    def command(self, action: int) -> VelocityCommand:
        if isinstance(action, bool) or not isinstance(action, int):
            raise TypeError('action must be an integer')
        if action < 0 or action >= self.size:
            raise ValueError(f'action must be in [0, {self.size})')
        linear_x, angular_z = self.config.commands[action]
        return VelocityCommand(linear_x, angular_z, self.config.duration_seconds)

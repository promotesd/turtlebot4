"""Action mapper protocol."""

from typing import Protocol

from turtlebot4_rl_core.types import VelocityCommand


class ActionMapper(Protocol):
    """Maps an agent action into a robot command."""

    @property
    def size(self) -> int: ...

    def command(self, action: int) -> VelocityCommand: ...

"""Robot I/O port."""

from typing import Protocol, runtime_checkable

from turtlebot4_rl_core.types import RobotObservation, VelocityCommand


@runtime_checkable
class RobotBackend(Protocol):
    """Provides sensor samples and executes safe velocity commands."""

    def is_ready(self) -> bool: ...

    def wait_until_ready(self, *, not_before_seconds: float = 0.0) -> bool:
        """Wait for a complete sample received after the requested boundary."""
        ...

    def read_observation(self) -> RobotObservation: ...

    def execute_action(self, command: VelocityCommand) -> None: ...

    def stop(self) -> None: ...

    def close(self) -> None: ...

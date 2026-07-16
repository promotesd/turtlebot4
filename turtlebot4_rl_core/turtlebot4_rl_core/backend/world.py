"""Episode-world control port."""

from typing import Protocol, runtime_checkable

from turtlebot4_rl_core.types import Goal2D


@runtime_checkable
class WorldBackend(Protocol):
    """Controls world state without exposing simulator details to the core."""

    @property
    def goal(self) -> Goal2D: ...

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, object] | None = None,
    ) -> dict[str, object]: ...

    def set_goal(self, x: float, y: float) -> None: ...

    def close(self) -> None: ...

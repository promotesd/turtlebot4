"""Minimal WorldBackend example for a future simulator adapter."""

from turtlebot4_rl_core import Goal2D


class ExampleWorldBackend:
    """Illustrate the port only; replace methods with real simulator calls."""

    def __init__(self) -> None:
        self._goal = Goal2D(1.0, 0.0)
        self._closed = False

    @property
    def goal(self) -> Goal2D:
        return self._goal

    def reset(self, *, seed=None, options=None):
        if self._closed:
            raise RuntimeError('backend is closed')
        options = options or {}
        goal = options.get('goal', (1.0, 0.0))
        self.set_goal(float(goal[0]), float(goal[1]))
        return {'seed': seed, 'backend': 'example'}

    def set_goal(self, x: float, y: float) -> None:
        self._goal = Goal2D(x, y)

    def close(self) -> None:
        self._closed = True

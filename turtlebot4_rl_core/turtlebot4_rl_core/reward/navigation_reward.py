"""Shaped point-goal navigation reward."""

from dataclasses import dataclass


@dataclass(frozen=True)
class NavigationRewardConfig:
    """Weights for transparent reward components."""

    progress_weight: float = 5.0
    step_penalty: float = -0.01
    goal_reward: float = 100.0
    collision_penalty: float = -100.0


class NavigationReward:
    """Reward progress while strongly separating terminal outcomes."""

    def __init__(self, config: NavigationRewardConfig | None = None) -> None:
        self.config = config or NavigationRewardConfig()
        self._previous_distance: float | None = None

    def reset(self, initial_distance: float) -> None:
        self._previous_distance = initial_distance

    def calculate(
        self, distance: float, *, reached_goal: bool, collided: bool
    ) -> tuple[float, dict[str, float]]:
        if self._previous_distance is None:
            raise RuntimeError('reward function must be reset before use')
        progress = self.config.progress_weight * (self._previous_distance - distance)
        goal = self.config.goal_reward if reached_goal else 0.0
        collision = self.config.collision_penalty if collided else 0.0
        components = {
            'progress': progress,
            'step': self.config.step_penalty,
            'goal': goal,
            'collision': collision,
        }
        self._previous_distance = distance
        return float(sum(components.values())), components

"""Point-goal navigation task composition."""

from dataclasses import dataclass, field

from turtlebot4_rl_core.action import DiscreteVelocityAction
from turtlebot4_rl_core.observation import LidarGoalObservation
from turtlebot4_rl_core.reward import NavigationReward
from turtlebot4_rl_core.termination import NavigationTermination


@dataclass
class PointGoalNavigationTask:
    """Default replaceable components for point-goal navigation."""

    observation: LidarGoalObservation = field(default_factory=LidarGoalObservation)
    action: DiscreteVelocityAction = field(default_factory=DiscreteVelocityAction)
    reward: NavigationReward = field(default_factory=NavigationReward)
    termination: NavigationTermination = field(default_factory=NavigationTermination)

"""Reproducible configuration-driven DQN training and evaluation command."""

import argparse
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from turtlebot4_dqn import DQNAgent, load_dqn_configuration
from turtlebot4_dqn.checkpoint import load_checkpoint, save_checkpoint
from turtlebot4_dqn.evaluator import evaluate
from turtlebot4_dqn.trainer import EpisodeMetrics, train
from turtlebot4_rl_bringup.configuration import load_configuration
from turtlebot4_rl_bringup.factory import default_registry


def summarize(metrics: list[EpisodeMetrics], mode: str) -> dict[str, object]:
    """Aggregate a non-empty metric set for stable JSON output."""
    return {
        'mode': mode,
        'episodes': len(metrics),
        'mean_reward': sum(item.reward for item in metrics) / len(metrics),
        'success_rate': sum(item.success for item in metrics) / len(metrics),
        'collision_rate': sum(item.collision for item in metrics) / len(metrics),
        'timeout_rate': sum(item.truncated for item in metrics) / len(metrics),
        'mean_duration_seconds': sum(item.duration_seconds for item in metrics) / len(metrics),
        'mean_path_length': sum(item.path_length for item in metrics) / len(metrics),
        'mean_spl': sum(item.spl for item in metrics) / len(metrics),
        'mean_inference_ms': sum(item.mean_inference_ms for item in metrics) / len(metrics),
        'seeds': [item.seed for item in metrics],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('train', 'evaluate'), default='train')
    parser.add_argument('--episodes', type=int, default=10)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--environment-config')
    parser.add_argument('--dqn-config')
    parser.add_argument('--checkpoint')
    args = parser.parse_args()
    environment_path = args.environment_config or (
        get_package_share_directory('turtlebot4_rl_bringup') + '/config/mock.yaml'
    )
    dqn_path = args.dqn_config or (
        get_package_share_directory('turtlebot4_dqn') + '/config/dqn.yaml'
    )
    framework_configuration = load_configuration(environment_path)
    dqn_configuration = load_dqn_configuration(dqn_path)
    environment = default_registry().create(framework_configuration)
    agent = DQNAgent(
        environment.observation_space.shape[0],
        environment.action_space.n,
        dqn_configuration.agent,
        seed=args.seed,
        exploration=dqn_configuration.exploration,
    )
    try:
        if args.mode == 'evaluate':
            if not args.checkpoint:
                parser.error('--checkpoint is required in evaluate mode')
            load_checkpoint(agent, args.checkpoint)
            metrics = evaluate(
                environment, agent, list(dqn_configuration.evaluation_seeds)
            )
        else:
            metrics = train(environment, agent, args.episodes, seed=args.seed)
            if args.checkpoint:
                save_checkpoint(agent, Path(args.checkpoint))
    finally:
        environment.close()
    print(json.dumps(summarize(metrics, args.mode), indent=2, sort_keys=True))


if __name__ == '__main__':
    main()

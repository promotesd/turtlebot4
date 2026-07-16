"""Reproducible configuration-driven DQN training and evaluation command."""

import argparse
import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory

from turtlebot4_dqn import DQNAgent, load_dqn_configuration
from turtlebot4_dqn.checkpoint import load_checkpoint, save_checkpoint
from turtlebot4_dqn.evaluator import evaluate
from turtlebot4_dqn.trainer import train
from turtlebot4_rl_bringup.configuration import load_configuration
from turtlebot4_rl_bringup.factory import default_registry
from turtlebot4_rl_bringup.reporting import (
    configuration_record,
    package_version,
    revision,
    summarize,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('train', 'evaluate'), default='train')
    parser.add_argument('--episodes', type=int, default=10)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--environment-config')
    parser.add_argument('--dqn-config')
    parser.add_argument('--checkpoint')
    parser.add_argument('--metrics-output')
    parser.add_argument('--revision')
    args = parser.parse_args()
    run_started = time.perf_counter()
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
    environment_config_path, environment_config_sha256 = configuration_record(
        environment_path
    )
    dqn_config_path, dqn_config_sha256 = configuration_record(dqn_path)
    backend = framework_configuration.backend
    report = summarize(
        metrics,
        args.mode,
        run_duration_seconds=time.perf_counter() - run_started,
        provenance={
            'code_revision': revision(args.revision),
            'environment_version': (
                f'turtlebot4_rl_core/{package_version("turtlebot4-rl-core")} '
                f'{backend.robot}+{backend.world}'
            ),
            'environment_config_path': environment_config_path,
            'environment_config_sha256': environment_config_sha256,
            'dqn_config_path': dqn_config_path,
            'dqn_config_sha256': dqn_config_sha256,
            'backend': {'robot': backend.robot, 'world': backend.world},
        },
    )
    serialized = json.dumps(report, indent=2, sort_keys=True)
    if args.metrics_output:
        destination = Path(args.metrics_output)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_suffix(destination.suffix + '.tmp')
        temporary.write_text(serialized + '\n', encoding='utf-8')
        temporary.replace(destination)
    print(serialized)


if __name__ == '__main__':
    main()

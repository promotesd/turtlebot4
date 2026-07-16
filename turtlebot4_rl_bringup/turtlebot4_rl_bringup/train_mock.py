"""Reproducible configuration-driven DQN training and evaluation command."""

import argparse
from dataclasses import asdict
import hashlib
from importlib import metadata
import json
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_share_directory

from turtlebot4_dqn import DQNAgent, load_dqn_configuration
from turtlebot4_dqn.checkpoint import load_checkpoint, save_checkpoint
from turtlebot4_dqn.evaluator import evaluate
from turtlebot4_dqn.trainer import EpisodeMetrics, train
from turtlebot4_rl_bringup.configuration import load_configuration
from turtlebot4_rl_bringup.factory import default_registry


def _configuration_record(path: str | Path) -> tuple[str, str]:
    resolved = Path(path).resolve()
    return str(resolved), hashlib.sha256(resolved.read_bytes()).hexdigest()


def _revision(override: str | None = None) -> str:
    if override:
        return override
    try:
        result = subprocess.run(
            ['git', 'rev-parse', 'HEAD'],
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return 'unknown'
    if result.returncode != 0:
        return 'unknown'
    revision = result.stdout.strip()
    try:
        status = subprocess.run(
            ['git', 'status', '--porcelain', '--untracked-files=no'],
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return revision
    return revision + ('-dirty' if status.returncode == 0 and status.stdout else '')


def _package_version(distribution: str) -> str:
    try:
        return metadata.version(distribution)
    except metadata.PackageNotFoundError:
        return 'source-checkout'


def summarize(
    metrics: list[EpisodeMetrics],
    mode: str,
    *,
    run_duration_seconds: float = 0.0,
    provenance: dict[str, object] | None = None,
) -> dict[str, object]:
    """Aggregate a non-empty metric set for stable JSON output."""
    if not metrics:
        raise ValueError('metrics must not be empty')
    successful = [item for item in metrics if item.success]
    losses = [item.mean_loss for item in metrics if item.mean_loss is not None]
    report: dict[str, object] = {
        'mode': mode,
        'episodes': len(metrics),
        'mean_reward': sum(item.reward for item in metrics) / len(metrics),
        'success_rate': sum(item.success for item in metrics) / len(metrics),
        'collision_rate': sum(item.collision for item in metrics) / len(metrics),
        'timeout_rate': sum(item.truncated for item in metrics) / len(metrics),
        'mean_duration_seconds': sum(item.duration_seconds for item in metrics) / len(metrics),
        'mean_steps': sum(item.steps for item in metrics) / len(metrics),
        'mean_time_to_goal_seconds': (
            sum(item.duration_seconds for item in successful) / len(successful)
            if successful
            else None
        ),
        'mean_path_length': sum(item.path_length for item in metrics) / len(metrics),
        'mean_spl': sum(item.spl for item in metrics) / len(metrics),
        'mean_inference_ms': sum(item.mean_inference_ms for item in metrics) / len(metrics),
        'mean_training_loss': sum(losses) / len(losses) if losses else None,
        'run_duration_seconds': run_duration_seconds,
        'seeds': [item.seed for item in metrics],
        'per_episode': [asdict(item) for item in metrics],
    }
    report[f'{mode}_duration_seconds'] = run_duration_seconds
    if provenance:
        report.update(provenance)
    return report


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
    environment_config_path, environment_config_sha256 = _configuration_record(
        environment_path
    )
    dqn_config_path, dqn_config_sha256 = _configuration_record(dqn_path)
    backend = framework_configuration.backend
    report = summarize(
        metrics,
        args.mode,
        run_duration_seconds=time.perf_counter() - run_started,
        provenance={
            'code_revision': _revision(args.revision),
            'environment_version': (
                f'turtlebot4_rl_core/{_package_version("turtlebot4-rl-core")} '
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

"""ROS-independent scientific report aggregation and provenance helpers."""

from dataclasses import asdict
import hashlib
from importlib import metadata
from pathlib import Path
import subprocess

from turtlebot4_dqn.trainer import EpisodeMetrics


def configuration_record(path: str | Path) -> tuple[str, str]:
    """Return an absolute configuration path and its SHA-256 digest."""
    resolved = Path(path).resolve()
    return str(resolved), hashlib.sha256(resolved.read_bytes()).hexdigest()


def revision(override: str | None = None) -> str:
    """Resolve the full Git revision and mark tracked local modifications."""
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
    resolved_revision = result.stdout.strip()
    try:
        status = subprocess.run(
            ['git', 'status', '--porcelain', '--untracked-files=no'],
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return resolved_revision
    return resolved_revision + ('-dirty' if status.returncode == 0 and status.stdout else '')


def package_version(distribution: str) -> str:
    """Return an installed distribution version or a source-tree marker."""
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

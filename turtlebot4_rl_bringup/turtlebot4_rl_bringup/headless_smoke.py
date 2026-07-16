"""Repeated reset verification against a running headless Gazebo world."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np

from turtlebot4_rl_bringup.configuration import load_configuration
from turtlebot4_rl_bringup.factory import default_registry


def _default_configuration() -> Path:
    return (
        Path(get_package_share_directory('turtlebot4_rl_bringup')) / 'config' / 'navigation.yaml'
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Verify repeated fresh-sensor resets in a running Gazebo world.'
    )
    parser.add_argument('--episodes', type=int, default=100)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--environment-config', type=Path, default=_default_configuration())
    parser.add_argument('--motion-action', type=int, default=2)
    parser.add_argument('--motion-steps', type=int, default=10)
    parser.add_argument('--motion-min-distance', type=float, default=0.002)
    parser.add_argument('--reset-pose-tolerance', type=float, default=0.02)
    args = parser.parse_args()
    if args.episodes <= 0:
        parser.error('--episodes must be positive')
    if args.motion_steps <= 0:
        parser.error('--motion-steps must be positive')
    if args.motion_min_distance <= 0.0:
        parser.error('--motion-min-distance must be positive')
    if args.reset_pose_tolerance <= 0.0:
        parser.error('--reset-pose-tolerance must be positive')

    configuration = load_configuration(args.environment_config)
    environment = default_registry().create(configuration)
    durations: list[float] = []
    reset_position_errors: list[float] = []
    expected_reset_x = float(configuration.gazebo.get('robot_start_x', 0.0))
    expected_reset_y = float(configuration.gazebo.get('robot_start_y', 0.0))
    try:
        if not 0 <= args.motion_action < environment.action_space.n:
            raise ValueError(
                f'motion action {args.motion_action} is outside '
                f'[0, {environment.action_space.n})'
            )
        _, motion_start = environment.reset(seed=args.seed - 1)
        start_x, start_y, _ = motion_start['robot_pose']
        motion_info = motion_start
        for motion_step in range(args.motion_steps):
            _, _, terminated, truncated, motion_info = environment.step(args.motion_action)
            if terminated or truncated:
                raise RuntimeError(
                    f'motion probe ended unexpectedly at step {motion_step}: '
                    f"{motion_info.get('termination_reason')}"
                )
        end_x, end_y, _ = motion_info['robot_pose']
        displacement = math.hypot(end_x - start_x, end_y - start_y)
        if displacement < args.motion_min_distance:
            raise RuntimeError(
                f'/cmd_vel motion probe moved {displacement:.4f} m; '
                f'expected at least {args.motion_min_distance:.4f} m'
            )
        print(f'/cmd_vel motion probe moved {displacement:.4f} m', flush=True)

        for episode in range(args.episodes):
            started = time.monotonic()
            try:
                observation, info = environment.reset(seed=args.seed + episode)
            except Exception as error:
                raise RuntimeError(f'Gazebo reset failed at episode {episode}') from error
            durations.append(time.monotonic() - started)
            if not environment.observation_space.contains(observation):
                raise RuntimeError(f'episode {episode} returned an invalid observation')
            if info.get('seed') != args.seed + episode:
                raise RuntimeError(f'episode {episode} returned the wrong seed')
            pose = info.get('robot_pose')
            if not isinstance(pose, tuple) or len(pose) != 3:
                raise RuntimeError(f'episode {episode} did not return a robot pose')
            position_error = math.hypot(
                float(pose[0]) - expected_reset_x,
                float(pose[1]) - expected_reset_y,
            )
            reset_position_errors.append(position_error)
            if position_error > args.reset_pose_tolerance:
                raise RuntimeError(
                    f'episode {episode} reset position error was {position_error:.4f} m; '
                    f'limit is {args.reset_pose_tolerance:.4f} m'
                )
            if (episode + 1) % 10 == 0 or episode + 1 == args.episodes:
                print(f'completed {episode + 1}/{args.episodes} resets', flush=True)
    finally:
        environment.close()

    print(
        json.dumps(
            {
                'episodes': args.episodes,
                'seed_start': args.seed,
                'motion_displacement_m': displacement,
                'mean_reset_seconds': float(np.mean(durations)),
                'max_reset_seconds': max(durations),
                'max_reset_position_error_m': max(reset_position_errors),
                'status': 'passed',
            },
            sort_keys=True,
        )
    )


if __name__ == '__main__':
    main()

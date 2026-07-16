"""Run a deterministic no-ROS environment smoke test."""

from turtlebot4_rl_mock import create_mock_environment


def main() -> None:
    env, _ = create_mock_environment()
    observation, info = env.reset(seed=42)
    total_reward = 0.0
    for _ in range(100):
        observation, reward, terminated, truncated, info = env.step(2)
        total_reward += reward
        if terminated or truncated:
            break
    print(
        f'steps={info["step"]} distance={info["goal_distance"]:.3f} '
        f'reward={total_reward:.3f} reason={info["termination_reason"]} '
        f'observation_size={observation.size}'
    )
    env.close()


if __name__ == '__main__':
    main()

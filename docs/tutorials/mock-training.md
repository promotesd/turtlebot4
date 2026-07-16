# Deterministic Mock training

Build and source the workspace, then run:

```bash
ros2 run turtlebot4_rl_mock mock_demo
ros2 run turtlebot4_rl_bringup train_mock --episodes 10 --seed 42
```

Mock provides fixed obstacles, optional seeded goal randomization, laser/odometry-like
state, collision/goal/time-limit outcomes, and fault injection. It is designed for API
and algorithm tests, not for claims about Gazebo or physical navigation quality.

For comparisons, record the Git revision, YAML configuration, explicit seed list,
episode count, reward, success/collision/timeout rates, mean steps, training time, and
inference latency. Run multiple seeds. Checkpoints contain network arrays and training
and environment step counts but not replay memory or environment state. Training and
evaluation report duration, path length, SPL, and mean inference latency per episode.
Epsilon decays by environment interactions, independently from gradient-update count.

Before using ROS/Gazebo, read the architecture document and validate that `/scan`,
`/odom`, `/cmd_vel`, world name, and entity names match configuration. Keep a simulator
test report distinct from the Mock report.

# TurtleBot 4 common and reinforcement learning packages

This repository contains the TurtleBot 4 common ROS 2 packages and an experimental,
backend-independent reinforcement learning framework for point-goal navigation.
The RL framework is an early community extension: its APIs may change before 1.0 and
it is not an official Clearpath Robotics or ROBOTIS product.

The design keeps the learning algorithm separate from ROS 2 and simulators. The same
environment API can run against a deterministic Mock backend, standard ROS 2 robot
topics, or Gazebo Harmonic world control. This lets contributors test reward,
termination, reproducibility, and DQN behavior without starting a simulator.

## Compatibility

| Component | Supported baseline |
| --- | --- |
| Ubuntu | 24.04 |
| ROS 2 | Jazzy |
| Python | 3.12 |
| Simulator | Gazebo Harmonic through `ros_gz` |
| Robot I/O | `LaserScan`, `Odometry`, `TwistStamped` |

The original TurtleBot 4 packages remain documented in the
[TurtleBot 4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_common.html).

## RL packages

- `turtlebot4_rl_interfaces`: stable ROS messages and reset/goal services.
- `turtlebot4_rl_core`: typed environment, tasks, rewards, termination, and ports;
  it does not import ROS, Gazebo, TensorFlow, or PyTorch.
- `turtlebot4_rl_mock`: deterministic in-process test backend.
- `turtlebot4_rl_ros`: standard ROS 2 sensor/command robot adapter with watchdogs.
- `turtlebot4_rl_gz`: Gazebo Harmonic world and entity reset adapter.
- `turtlebot4_dqn`: replay, exploration, checkpoints, training, evaluation, and
  interchangeable NumPy/TensorFlow Q-networks.
- `turtlebot4_rl_bringup`: launch files and configuration.
- `turtlebot4_machine_learning`: convenience metapackage.

See [the architecture](docs/architecture/rl-framework.md),
[the RFC](docs/design/rl-framework-rfc.md), and the
[Mock tutorial](docs/tutorials/mock-training.md) before extending a backend. New
adapters use the [backend registry](docs/tutorials/custom-backend.md), so adding a
simulator does not require changes in Core or DQN.
The [verification matrix](docs/design/verification-matrix.md) distinguishes completed
local evidence from simulator, hardware, and GitHub gates that require external state.

## Install and build

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash
```

The framework's minimal Core/Mock/DQN path needs only Python, NumPy, PyYAML, and
pytest. TensorFlow is optional and imported only when `TensorFlowQNetwork` is selected.

## Run the minimum example

```bash
source install/setup.bash
ros2 run turtlebot4_rl_mock mock_demo
ros2 run turtlebot4_rl_bringup train_rl --episodes 10 --seed 42 \
  --checkpoint /tmp/turtlebot4_dqn.npz
```

The first command runs a fixed policy. The second prints machine-readable training
metrics, including reward, outcome rates, path length, SPL, duration, and inference
latency. Both load validated YAML rather than fixed state/action sizes. Neither command
starts ROS middleware or Gazebo internally.

## Test

```bash
colcon test --packages-select \
  turtlebot4_rl_core turtlebot4_rl_mock turtlebot4_rl_ros turtlebot4_rl_gz \
  turtlebot4_dqn turtlebot4_rl_bringup
colcon test-result --verbose

# Fast source-tree suite
PYTHONPATH=turtlebot4_rl_core:turtlebot4_rl_mock:turtlebot4_rl_ros:turtlebot4_rl_gz:turtlebot4_dqn:turtlebot4_rl_bringup \
  pytest -q turtlebot4_rl_core/test turtlebot4_rl_mock/test \
  turtlebot4_rl_ros/test turtlebot4_rl_gz/test turtlebot4_dqn/test \
  turtlebot4_rl_bringup/test
```

Gazebo and physical-robot checks are deliberately separate because they require
external processes or hardware. Never report those checks as passing based only on
Mock results.

## Safety

Physical-robot use must begin with sensor-only and zero-velocity checks. The ROS
adapter stops on command timeout, stale sensors, episode end, exceptions, and close.
Use an accessible hardware emergency stop and human supervision. World teleport/reset
operations belong to simulator backends and must not be enabled on a physical robot.

## Contributing and support

Read [CONTRIBUTING.md](CONTRIBUTING.md), the [roadmap](ROADMAP.md), and
[SUPPORT.md](SUPPORT.md). Report vulnerabilities privately according to
[SECURITY.md](SECURITY.md). Community conduct is governed by
[CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md). The project is Apache-2.0 licensed; derived
work and provenance are recorded in [NOTICE](NOTICE).

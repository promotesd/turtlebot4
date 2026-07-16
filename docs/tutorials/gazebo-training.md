# Gazebo Harmonic training

This repository contains the adapter, not the TurtleBot 4 simulation worlds. First
install and launch a compatible `turtlebot4_simulator` world whose configured world
name is `depot` and robot entity is `turtlebot4`. Verify `/scan`, `/odom`, `/cmd_vel`,
and the `/world/depot/*` ROS-Gazebo services before training.

With the simulator already running:

```bash
ros2 launch turtlebot4_rl_bringup gazebo_training.launch.py \
  episodes:=10 checkpoint:=/tmp/turtlebot4_dqn.npz
```

For evaluation, use the installed generic command and fixed seeds from `dqn.yaml`:

```bash
ros2 run turtlebot4_rl_bringup train_rl --mode evaluate \
  --environment-config "$(ros2 pkg prefix turtlebot4_rl_bringup)/share/turtlebot4_rl_bringup/config/navigation.yaml" \
  --dqn-config "$(ros2 pkg prefix turtlebot4_dqn)/share/turtlebot4_dqn/config/dqn.yaml" \
  --checkpoint /tmp/turtlebot4_dqn.npz
```

The adapter pauses the configured world, resets the robot pose, replaces the goal
marker, resumes, and waits for settling. A valid release report must still demonstrate
fresh sensors and 100 repeated resets in a pinned simulator version; adapter unit tests
and Mock episodes are not substitutes.

# Gazebo Harmonic training

Install the Jazzy TurtleBot 4 simulator and build this workspace. The supported
baseline uses the simulator's `warehouse` world, robot entity `turtlebot4`, `/scan`,
`/cmd_vel`, and `/sim_ground_truth_pose`. The latter is intentional: Gazebo entity
teleportation does not reset the diff-drive controller's accumulated `/odom`, while
the ground-truth topic remains in the same world frame as randomized goals.

For the normal interactive simulator, start TurtleBot 4 without Nav2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py \
  world:=warehouse rviz:=false
```

Then train in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtlebot4_rl_bringup gazebo_training.launch.py \
  episodes:=10 checkpoint:=/tmp/turtlebot4_dqn.npz
```

## Headless reset verification

The release gate runs Gazebo's server, clock bridge, and robot spawn in separate
terminals. Use the same `ROS_DOMAIN_ID` in every terminal. The first terminal starts
the world without a GUI:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOG_DIR=/tmp/turtlebot4_ros_logs
export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/turtlebot4_gz_bringup/worlds:/opt/ros/jazzy/share/irobot_create_gz_bringup/worlds:/opt/ros/jazzy/share"
ros2 launch ros_gz_sim gz_sim.launch.py \
  gz_args:='-r -s --headless-rendering warehouse.sdf'
```

The second terminal bridges simulation time, and the third spawns the Lite model. The
reset contract only needs lidar, odometry, and velocity control, so the Lite model
avoids unrelated display/camera load on CPU-only CI hosts:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch turtlebot4_gz_bringup turtlebot4_spawn.launch.py \
  rviz:=false model:=lite
```

Run the lidar bridge below in a fourth terminal. The stock spawn launch starts many
bridge processes; dedicating one process to the reset gate prevents an unrelated bridge
failure from hiding behind an existing but inactive `/scan` topic:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
scan_topic=/world/warehouse/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan
ros2 run ros_gz_bridge parameter_bridge \
  "${scan_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" \
  --ros-args -r "${scan_topic}:=/scan"
```

After `/scan`, `/sim_ground_truth_pose`, and `/cmd_vel` publish, run the deterministic
100-reset gate from a fifth terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run turtlebot4_rl_bringup headless_reset_smoke \
  --episodes 100 --seed 42
```

The gate first sends ten low-speed forward actions and requires at least 0.002 m of
ground-truth displacement, proving the rate-limited `/cmd_vel` stream reaches the
simulated drivetrain even when a CPU-only server runs far below real time. This is a
connectivity smoke test, not a velocity benchmark. The following reset publishes zero
velocity. Each reset then pauses the world, resets the robot pose, replaces the goal
marker, resumes the world, and waits for a complete sensor sample received after reset.
It also requires the reported reset position to be within 0.02 m of the configured
start. Gazebo service calls have bounded timeouts and retries. The command exits
non-zero with the failing step or episode number.

The acceptance run on Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic completed 100 of
100 resets with seed 20260716. Mean reset time was 0.322 seconds and the maximum was
0.388 seconds; every observation used lidar and pose data received after its reset, and
the maximum reset-position error was 0.0093 m. The low-speed command probe observed
0.0261 m of motion followed by a zero-speed reset. This run enabled `/clock` and source
timestamp validation for both sensor streams.

## Optional obstacle randomization

Obstacle randomization is disabled by default. Enable it only for model entities that
the selected world permits Gazebo to move:

```yaml
gazebo:
  randomize_obstacles: true
  randomizable_obstacles: [barrier_0, barrier_1]
  obstacle_min_x: -2.5
  obstacle_max_x: 2.5
  obstacle_min_y: -2.0
  obstacle_max_y: 2.0
  obstacle_clearance: 0.6
  obstacle_sample_attempts: 100
```

Sampling uses the episode seed and keeps every configured obstacle away from the robot
start, goal, and previously sampled obstacle centers. Failure to find a valid pose is
an explicit reset error rather than a partially randomized episode.

## Evaluation

Evaluate a frozen checkpoint separately from training:

```bash
ros2 run turtlebot4_rl_bringup train_rl --mode evaluate \
  --environment-config "$(ros2 pkg prefix turtlebot4_rl_bringup)/share/turtlebot4_rl_bringup/config/navigation.yaml" \
  --dqn-config "$(ros2 pkg prefix turtlebot4_dqn)/share/turtlebot4_dqn/config/dqn.yaml" \
  --checkpoint /tmp/turtlebot4_dqn.npz
```

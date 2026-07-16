# RL framework architecture

```text
trainer / evaluator / DQN
             |
             v
     turtlebot4_rl_core
       |             |
 RobotBackend    WorldBackend
   |     |          |      |
 Mock   ROS 2      Mock   Gazebo Harmonic
```

Dependencies point inward. Core owns observations, action mapping, reward, termination,
and episode state. A robot adapter owns sensor acquisition and safe command execution.
A world adapter owns goal/entity/reset operations. The paired Mock adapters share a
small deterministic simulation solely for tests.

Bringup owns a registry keyed by `(robot_backend, world_backend)`. Its built-in pairs
are `mock/mock` and `ros2/gazebo_harmonic`; downstream packages can register another
pair without editing algorithms or Core. YAML is converted into validated observation,
action, reward, termination, ROS, and Gazebo configuration before construction.

The environment stops the robot before reset, at either terminal flag, on action/read
exceptions, and on close. The ROS adapter adds independent freshness and command
watchdogs. `close()` is idempotent at every boundary.

To add a simulator, implement `WorldBackend`, keep reset/entity names in configuration,
and pass the shared contract tests. To add a robot transport, implement `RobotBackend`
and prove that stale data, timeouts, exceptions, and shutdown publish a safe stop. Do
not add backend imports or backend-specific names to Core or DQN.

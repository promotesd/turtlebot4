# RFC: backend-independent reinforcement learning for TurtleBot 4

- Status: implemented for community review
- Target: ROS 2 Jazzy
- Scope: Core, interfaces, Mock, standard ROS, Gazebo adapter, DQN baseline

## Problem

A reinforcement-learning experiment that mixes neural networks, rewards, ROS callbacks,
and simulator reset calls cannot be tested deterministically or moved safely between a
simulator and a robot. The TurtleBot 4 common repository also cannot assume that every
user installs a deep-learning framework or Gazebo.

## Goals

- Provide a Gymnasium-style `reset`/`step` environment with explicit `terminated` and
  `truncated` values.
- Isolate algorithms, task semantics, robot I/O, and world control behind typed ports.
- Support deterministic no-ROS tests, standard ROS topics, and Gazebo Harmonic.
- Provide a modest DQN reference with reproducible seeds and portable checkpoints.
- Preserve all existing TurtleBot 4 common packages and their users.

## Non-goals

This RFC does not promise an optimal policy, a trained model, unsupervised physical-
robot training, simulator parity, or acceptance into an official TurtleBot repository.
PPO/SAC, distributed training, perception models, and automatic world-wide domain
randomization are future work. The Gazebo adapter supports opt-in seeded pose
randomization for an explicit allow-list of movable obstacle entities.

## User scenarios

A contributor can test a reward in milliseconds with Mock, train against Gazebo without
changing DQN, evaluate a frozen policy over multiple seeds, implement another
`WorldBackend`, or connect the standard ROS robot adapter under human supervision.

## TurtleBot3 lessons

Earlier examples commonly combined environment services, Gazebo entity names, a
single `done` flag, fixed state/action sizes, TensorFlow network construction, and
training control. This design uses TurtleBot3 only as a conceptual reference and does
not copy its machine-learning sources. Ports and configuration replace those couplings.

## Architecture and package boundaries

Algorithms depend on `turtlebot4_rl_core`; Core depends only on Python and NumPy. The
Mock, ROS, and Gazebo packages implement ports in the opposite direction. DQN does not
import any backend. ROS interfaces are separate so message generation does not pull
learning frameworks into robot packages. See `docs/architecture/rl-framework.md`.

## ROS 2 interfaces

Robot I/O consumes `sensor_msgs/LaserScan` and `nav_msgs/Odometry`, and publishes
`geometry_msgs/TwistStamped`. Topic/frame names, freshness, and command watchdogs are
configured. `ResetEpisode`, `SetGoal`, and `EpisodeState` provide integration surfaces
without encoding DQN internals.

## Environment API

`reset(seed, options)` returns `(observation, info)`. `step(action)` returns
`(observation, reward, terminated, truncated, info)`. Goal and collision are task
termination; time limit, stale sensors, and backend faults are truncation. Observations
and action spaces are derived from validated configuration rather than constants.

## Simulator and robot backends

`RobotBackend` supplies sensor samples, executes velocity commands, stops, and closes.
`WorldBackend` resets episode state and manages a goal. Gazebo owns pause/entity calls;
the standard ROS backend owns topics and safe stopping. Mock implements both over a
shared deterministic state. A future simulator implements only the relevant port.

## Physical-robot safety boundary

Physical mode never teleports the robot or resets physics. Sensor freshness and command
timeouts produce zero velocity. Exceptions, episode end, and close also stop. Validation
progresses through sensor-only, zero command, one low-speed action, short supervised
episodes, then fixed-goal evaluation with an accessible emergency stop.

## Dependencies and configuration

Core requires NumPy. DQN's reference network is NumPy; TensorFlow is an optional lazy
adapter. ROS/Gazebo dependencies stay in adapter packages. Observation bins/ranges,
commands, thresholds, topics, entity names, timeouts, goals, and DQN hyperparameters
are validated configuration values.

## Licensing and provenance

New files are Apache-2.0. Existing TurtleBot 4 notices are preserved. No TurtleBot3 ML
source was copied. Future derived files must preserve their original copyright/license
headers and be recorded in `NOTICE` with repository, revision, and modifications.

## Test strategy

Pure unit tests cover geometry, scan processing, actions, reward, termination, replay,
epsilon, learning, and checkpoints. Contract-style Mock tests cover reset, stop,
faults, close, 1,000 resets, and seed determinism. ROS sensor conversions are pure
tests. Gazebo headless and real-robot checks remain explicit, environment-dependent
jobs and must never be inferred from Mock success.

## Performance and scientific metrics

Evaluation records revision, config, environment version, seed set, reward, success,
collision, timeout, steps, elapsed time, path length, SPL, training duration, and
inference latency. The initial target is zero uncaught errors over 1,000 Mock resets and
deterministic trajectories for equal seeds. Simulator targets are 100 repeated resets
and fresh `/scan`/pose data. Reset and step both gate observations on samples received
after the world reset or action boundary. The Harmonic `warehouse` acceptance run
completed 100 of 100 resets with post-reset, source-timestamp-valid sensor samples
(mean 0.322 seconds, maximum 0.388 seconds) and observed 0.0261 m of low-speed command
motion before zero-stop.

## Pull-request split

1. RFC/ADRs and governance.
2. Interfaces, Core, Mock, and tests.
3. Standard ROS adapter and safety behavior.
4. DQN baseline.
5. Gazebo adapter in the simulator repository if upstream requests it.
6. Tutorials, metrics, and evidence.

The community fork may integrate these for learning and demonstration, but upstream
proposals should remain focused draft PRs.

## Alternatives

A monolithic ROS/Gazebo training node is smaller initially but untestable without its
runtime. A simulator-specific Gym environment duplicates algorithm/task behavior.
Making TensorFlow mandatory increases installation and CI cost. A generic robotics RL
framework would expand scope beyond TurtleBot 4. These alternatives were rejected.

## Open questions

- Whether official maintainers want Core/interfaces here or in a dedicated ML repository.
- Whether Gazebo integration belongs exclusively in `turtlebot4_simulator`.
- Which stable plugin discovery mechanism and metrics schema should reach 1.0.
- Which reference world/config and seed set should define algorithm regressions.

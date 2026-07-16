# Roadmap

## 0.1 — deterministic foundation

- Backend-independent environment and contracts.
- Mock, standard ROS 2, and Gazebo Harmonic adapters.
- NumPy DQN baseline, tests, tutorials, and CI.

## 0.2 — simulator evidence

- TurtleBot 4 simulator Stage 1 launch integration.
- Repeated headless resets, fresh-sensor barriers, and recorded performance baselines.
- Backend discovery through Python entry points.

## 0.3 — supervised robot evaluation

- Sensor-only and zero-command validation checklist.
- Low-speed, human-supervised evaluation with explicit emergency-stop procedures.
- Stable configuration and metrics schemas.

## 1.0 — stable interfaces

- Maintainer-reviewed ROS/core API stability policy.
- Multi-seed reproducibility report and versioned reference environments.
- Decision on whether algorithms remain here or move to a dedicated repository.

The roadmap is directional, not a promise of dates. Simulator packages may be proposed
to `turtlebot4_simulator`; official placement requires upstream maintainer agreement.

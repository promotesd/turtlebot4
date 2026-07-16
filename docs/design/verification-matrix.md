# Verification matrix

This matrix separates evidence produced in this checkout from environment-dependent
work that must not be inferred. Commands were run on Ubuntu 24.04 with ROS 2 Jazzy.

| Requirement | Evidence | Status |
| --- | --- | --- |
| Core has no ROS/simulator/framework dependency | import scan, package metadata, strict mypy | Verified |
| DQN has no backend dependency | import scan, package metadata, strict mypy | Verified |
| Gymnasium reset/step termination semantics | Core unit tests and Mock episodes | Verified |
| Configurable observation/action/reward/termination | YAML loader tests and installed CLI | Verified |
| Replaceable backend pairs | registry tests for built-in and custom pairs | Verified |
| Deterministic Mock and no episode leak | equal-seed trajectory plus 1,000 complete episodes | Verified |
| ROS sensor/command safety | live local DDS test for scan/odom/cmd, stale data, watchdog zero | Verified |
| Gazebo adapter API and goal SDF | package build and valid SDF/config unit tests | Verified |
| Gazebo headless repeated reset | requires pinned `turtlebot4_simulator` world | Not run |
| DQN replay/target/update/checkpoint | unit tests, checkpoint round trip | Verified |
| Train/evaluate separation and multi-seed metrics | installed `train_rl` train and 5-seed evaluate runs | Verified |
| Reward/outcome/path/SPL/timing metrics | trainer/evaluator tests and CLI JSON output | Verified |
| Full new-package ROS build | 8 selected packages finished | Verified |
| New-package colcon tests | 25 tests, 0 errors/failures/skips | Verified |
| Ruff/mypy/ament/Python/YAML/CMake/docs checks | local commands and link checker | Verified |
| Physical-robot staged safety validation | requires TurtleBot 4 and human supervision | Not run |
| Fork Draft PR and GitHub Actions | requires authenticated GitHub CLI/SSH | Pending |
| Official upstream RFC/PR | requires fork PR evidence and maintainer coordination | Pending |

The last four external gates are release/coordination evidence, not unit-test targets.
They remain explicit so Mock success cannot be presented as Gazebo, hardware, or GitHub
success.

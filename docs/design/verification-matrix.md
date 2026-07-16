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
| ROS timestamp and clock-jump handling | old/future stamp tests, monotonic watchdog, cache clear and zero-stop on backward jump | Verified |
| Gazebo adapter API and goal SDF | package build and valid SDF/config unit tests | Verified |
| Seeded obstacle randomization | deterministic allow-list sampling, bounds and clearance unit test | Verified |
| Gazebo headless repeated reset | Harmonic `warehouse`, Lite model, `/cmd_vel` moved 0.0261 m, zero-stop, and 100/100 timestamp-valid resets; mean 0.322 s, max 0.388 s | Verified |
| DQN replay/target/update/checkpoint | unit tests, checkpoint round trip | Verified |
| Train/evaluate separation and multi-seed metrics | installed `train_rl` train and 5-seed evaluate runs | Verified |
| Reproducible scientific metrics | revision/config hashes, environment version, per-seed reward/outcome/steps/time/path/SPL/loss/inference JSON | Verified |
| Full new-package ROS build | 8 selected packages finished | Verified |
| New-package colcon tests | 31 tests, 0 errors/failures/skips | Verified |
| Ruff/mypy/ament/Python/YAML/CMake/docs checks | local commands and link checker | Verified |
| Physical-robot staged safety validation | requires TurtleBot 4 and human supervision | Not run |
| Fork Draft PR | `promotesd/turtlebot4` PR #1 from `agent/rl-framework` | Verified |
| GitHub Actions | final remote check read awaits explicit authorization | Pending |
| Official upstream RFC/PR | requires fork PR evidence and maintainer coordination | Pending |

The remaining external gates are release/coordination evidence, not unit-test targets.
They remain explicit so local or simulator success cannot be presented as hardware,
GitHub Actions, or upstream-maintainer approval.

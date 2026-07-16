# Contributing

Thank you for improving TurtleBot 4. Discuss major behavior, package boundaries, new
simulator dependencies, or public interfaces in an issue or draft RFC pull request
before implementation.

## Development workflow

1. Branch from the current `jazzy` branch using a descriptive feature branch.
2. Keep Core free of ROS, simulator, and deep-learning framework imports.
3. Add unit tests and the shared backend contract tests for behavior changes.
4. Run `git diff --check`, the source pytest suite, `colcon build`, and `colcon test`.
5. Open a focused draft pull request with motivation, architecture, compatibility,
   test evidence, and attribution.

Do not commit credentials, recorded sensor datasets, generated build trees, model
weights, or training logs. New parameters need validation and documentation; avoid
robot names, world names, topic names, observation sizes, and action sizes in code.

## Code and documentation

Python code uses type hints, docstrings, and dependency injection. Public behavior
must distinguish task termination from external truncation. Tests must be deterministic
and must not claim simulator or hardware coverage unless those systems actually ran.

Copied or modified third-party code must retain its copyright and license headers.
Record the upstream path, revision, license, and modifications in `NOTICE`. A conceptual
reimplementation should not claim to be copied code.

By contributing, you agree that your contribution is licensed under Apache-2.0 and
that you will follow the Code of Conduct.

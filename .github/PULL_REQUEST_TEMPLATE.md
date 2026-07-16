## Summary

## Motivation

## Architecture and scope

## Non-goals

## Package changes

## Testing

List exact commands and environment. Distinguish Mock, Gazebo, and physical-robot evidence.

## Compatibility and safety

## Licensing and attribution

## Documentation and follow-up

## Checklist

- [ ] The diff is focused and has no credentials, generated models, logs, or datasets.
- [ ] Core does not import ROS, Gazebo, or a deep-learning framework.
- [ ] New behavior has deterministic tests and validated configuration.
- [ ] `terminated` and `truncated` semantics remain distinct.
- [ ] Robot command paths safely stop on timeout, errors, episode end, and shutdown.
- [ ] Third-party provenance and license headers are preserved and recorded.
- [ ] I ran `git diff --check`, relevant pytest/colcon checks, and documented limits.

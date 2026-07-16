# Add a backend

Implement `RobotBackend` for a new sensor/command transport or `WorldBackend` for a new
simulator. Keep simulator calls, entity names, and transport imports in the adapter.
The minimal world shape is shown in
[`examples/custom_world_backend.py`](../../examples/custom_world_backend.py).

Register an assembled backend pair in Bringup:

```python
registry = default_registry()
registry.register('ros2', 'my_simulator', create_my_environment)
environment = registry.create(configuration)
```

The factory receives validated configuration and returns the Gymnasium-style
environment. It must own and close any executor threads. Add the same reset, step,
determinism, safe-stop, repeated-close, timeout, and error contract checks used by
Mock/ROS. A simulator adapter must also prove repeated headless reset against a pinned
world; a unit test cannot replace that evidence.

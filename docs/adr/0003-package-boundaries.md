# ADR 0003: enforce package boundaries

- Status: accepted

`turtlebot4_dqn` depends on Core, and Core depends on Python/NumPy only. ROS messages,
standard topics, Gazebo services, Mock simulation, launch, and algorithms live in
separate packages. Optional frameworks are lazy adapters. This costs more package
metadata but makes dependency violations visible to review and CI.

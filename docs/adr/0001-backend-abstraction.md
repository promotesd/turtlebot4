# ADR 0001: split robot and world backend ports

- Status: accepted

Robot I/O and episodic world control change for different reasons. Core therefore
depends on separate `RobotBackend` and `WorldBackend` protocols. This permits a real
robot plus manual world, ROS robot plus Gazebo world, or paired Mock state. It adds two
small interfaces but prevents simulator entity operations from reaching algorithms.

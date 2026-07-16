"""Thread-safe standard ROS 2 implementation of RobotBackend."""

from __future__ import annotations

from threading import Lock
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from turtlebot4_rl_core import Pose2D, RobotObservation, VelocityCommand
from turtlebot4_rl_ros.config import RosRobotConfig
from turtlebot4_rl_ros.sensor import quaternion_to_yaw, validate_ranges


class RosRobotBackend:
    """Consume LaserScan/Odometry and publish guarded TwistStamped commands."""

    def __init__(self, config: RosRobotConfig | None = None, node: Node | None = None) -> None:
        self.config = config or RosRobotConfig()
        if node is None and not rclpy.ok():  # type: ignore[attr-defined]
            rclpy.init()
        self._owns_node = node is None
        self.node = node or Node('turtlebot4_rl_robot_backend')
        self._lock = Lock()
        self._ranges: np.ndarray | None = None
        self._pose: Pose2D | None = None
        self._scan_received = 0.0
        self._odom_received = 0.0
        self._last_command = 0.0
        self._closed = False
        self._publisher = self.node.create_publisher(TwistStamped, self.config.cmd_vel_topic, 10)
        self._scan_subscription = self.node.create_subscription(
            LaserScan, self.config.scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self._odom_subscription = self.node.create_subscription(
            Odometry, self.config.odom_topic, self._on_odom, qos_profile_sensor_data
        )
        timer_period = min(self.config.command_timeout_seconds / 2.0, 0.1)
        self._watchdog = self.node.create_timer(timer_period, self._on_watchdog)

    def _on_scan(self, message: LaserScan) -> None:
        try:
            ranges = validate_ranges(message.ranges)
        except ValueError:
            return
        with self._lock:
            self._ranges = ranges
            self._scan_received = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        try:
            yaw = quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        except ValueError:
            return
        with self._lock:
            self._pose = Pose2D(position.x, position.y, yaw)
            self._odom_received = time.monotonic()

    def _publish(self, linear_x: float, angular_z: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = self.config.base_frame
        message.twist.linear.x = float(linear_x)
        message.twist.angular.z = float(angular_z)
        self._publisher.publish(message)

    def _on_watchdog(self) -> None:
        command_expired = (
            self._last_command
            and time.monotonic() - self._last_command > self.config.command_timeout_seconds
        )
        if command_expired:
            self.stop()

    def is_ready(self) -> bool:
        with self._lock:
            return not self._closed and self._ranges is not None and self._pose is not None

    def read_observation(self) -> RobotObservation:
        now = time.monotonic()
        with self._lock:
            if self._ranges is None or self._pose is None:
                raise RuntimeError('LaserScan and Odometry have not both been received')
            age = max(now - self._scan_received, now - self._odom_received)
            healthy = age <= self.config.sensor_timeout_seconds
            status = 'ok' if healthy else 'stale_sensor_data'
            return RobotObservation(self._ranges.copy(), self._pose, now, healthy, status)

    def execute_action(self, command: VelocityCommand) -> None:
        if self._closed:
            raise RuntimeError('ROS robot backend is closed')
        self._publish(command.linear_x, command.angular_z)
        self._last_command = time.monotonic()
        time.sleep(command.duration_seconds)

    def stop(self) -> None:
        if self._closed:
            return
        self._publish(0.0, 0.0)
        self._last_command = 0.0

    def close(self) -> None:
        if self._closed:
            return
        self.stop()
        self._closed = True
        if self._owns_node:
            self.node.destroy_node()  # type: ignore[no-untyped-call]

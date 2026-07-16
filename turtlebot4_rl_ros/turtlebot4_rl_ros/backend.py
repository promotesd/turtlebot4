"""Thread-safe standard ROS 2 implementation of RobotBackend."""

from __future__ import annotations

from threading import Condition, Lock
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
from numpy.typing import NDArray
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from turtlebot4_rl_core import Pose2D, RobotObservation, VelocityCommand
from turtlebot4_rl_ros.config import RosRobotConfig
from turtlebot4_rl_ros.sensor import (
    clock_moved_backwards,
    message_stamp_is_fresh,
    quaternion_to_yaw,
    validate_ranges,
)


class RosRobotBackend:
    """Consume LaserScan/Odometry and publish guarded TwistStamped commands."""

    def __init__(self, config: RosRobotConfig | None = None, node: Node | None = None) -> None:
        self.config = config or RosRobotConfig()
        if node is None and not rclpy.ok():
            rclpy.init()
        self._owns_node = node is None
        self.node = node or Node(
            'turtlebot4_rl_robot_backend',
            parameter_overrides=[Parameter('use_sim_time', value=self.config.use_sim_time)],
        )
        self._lock = Lock()
        self._sample_available = Condition(self._lock)
        self._ranges: NDArray[np.float32] | None = None
        self._pose: Pose2D | None = None
        self._scan_received = 0.0
        self._odom_received = 0.0
        self._last_command = 0.0
        self._last_ros_time_nanoseconds = 0
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

    def _prepare_message(self, stamp_seconds: int, stamp_nanoseconds: int) -> bool:
        now_nanoseconds = self.node.get_clock().now().nanoseconds
        jumped_backwards = False
        with self._sample_available:
            if clock_moved_backwards(
                self._last_ros_time_nanoseconds,
                now_nanoseconds,
                tolerance_seconds=self.config.clock_jump_tolerance_seconds,
            ):
                self._ranges = None
                self._pose = None
                self._scan_received = 0.0
                self._odom_received = 0.0
                jumped_backwards = True
                self._sample_available.notify_all()
            self._last_ros_time_nanoseconds = now_nanoseconds
        if jumped_backwards:
            self.stop()
        return message_stamp_is_fresh(
            stamp_seconds,
            stamp_nanoseconds,
            now_nanoseconds=now_nanoseconds,
            max_age_seconds=self.config.sensor_timeout_seconds,
            future_tolerance_seconds=self.config.future_timestamp_tolerance_seconds,
            allow_zero=self.config.allow_zero_message_timestamps,
        )

    def _on_scan(self, message: LaserScan) -> None:
        if not self._prepare_message(message.header.stamp.sec, message.header.stamp.nanosec):
            return
        try:
            ranges = validate_ranges(message.ranges)
        except ValueError:
            return
        with self._lock:
            self._ranges = ranges
            self._scan_received = time.monotonic()
            self._sample_available.notify_all()

    def _on_odom(self, message: Odometry) -> None:
        if not self._prepare_message(message.header.stamp.sec, message.header.stamp.nanosec):
            return
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        try:
            yaw = quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        except ValueError:
            return
        with self._lock:
            self._pose = Pose2D(position.x, position.y, yaw)
            self._odom_received = time.monotonic()
            self._sample_available.notify_all()

    def _publish(self, linear_x: float, angular_z: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = self.config.base_frame
        message.twist.linear.x = float(linear_x)
        message.twist.angular.z = float(angular_z)
        self._publisher.publish(message)

    def _on_watchdog(self) -> None:
        now = time.monotonic()
        with self._lock:
            command_active = bool(self._last_command)
            command_expired = (
                command_active and now - self._last_command > self.config.command_timeout_seconds
            )
            newest_sensor = min(self._scan_received, self._odom_received)
            sensor_stale = command_active and (
                newest_sensor <= 0.0 or now - newest_sensor > self.config.sensor_timeout_seconds
            )
        if command_expired or sensor_stale:
            self.stop()

    def is_ready(self) -> bool:
        with self._lock:
            return not self._closed and self._ranges is not None and self._pose is not None

    def wait_until_ready(self, *, not_before_seconds: float = 0.0) -> bool:
        deadline = time.monotonic() + self.config.readiness_timeout_seconds
        with self._sample_available:
            while not self._closed:
                complete = self._ranges is not None and self._pose is not None
                fresh = (
                    self._scan_received >= not_before_seconds
                    and self._odom_received >= not_before_seconds
                )
                if complete and fresh:
                    return True
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._sample_available.wait(timeout=remaining)
        return False

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
        deadline = time.monotonic() + command.duration_seconds
        period = 1.0 / self.config.command_publish_hz
        while True:
            now = time.monotonic()
            with self._lock:
                oldest_sensor = min(self._scan_received, self._odom_received)
                sensors_fresh = (
                    oldest_sensor > 0.0
                    and now - oldest_sensor <= self.config.sensor_timeout_seconds
                )
            if not sensors_fresh:
                self.stop()
                raise RuntimeError('cannot execute action with stale sensor data')
            self._publish(command.linear_x, command.angular_z)
            with self._lock:
                self._last_command = now
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(period, remaining))

    def stop(self) -> None:
        if self._closed:
            return
        self._publish(0.0, 0.0)
        with self._lock:
            self._last_command = 0.0

    def close(self) -> None:
        if self._closed:
            return
        self.stop()
        with self._sample_available:
            self._closed = True
            self._sample_available.notify_all()
        if self._owns_node:
            self.node.destroy_node()

from threading import Thread
import time
import uuid

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from turtlebot4_rl_core import VelocityCommand
from turtlebot4_rl_ros.backend import RosRobotBackend
from turtlebot4_rl_ros.config import RosRobotConfig


def wait_until(predicate, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError('condition did not become true before timeout')
        time.sleep(0.01)


@pytest.mark.integration
def test_ros_backend_topics_freshness_and_command_watchdog() -> None:
    rclpy.init()
    suffix = uuid.uuid4().hex
    namespace = f'/turtlebot4_rl_test_{suffix}'
    config = RosRobotConfig(
        scan_topic=namespace + '/scan',
        odom_topic=namespace + '/odom',
        cmd_vel_topic=namespace + '/cmd_vel',
        sensor_timeout_seconds=0.08,
        command_timeout_seconds=0.5,
    )
    backend = RosRobotBackend(config)
    driver = Node('turtlebot4_rl_test_driver_' + suffix)
    scan_publisher = driver.create_publisher(
        LaserScan, config.scan_topic, qos_profile_sensor_data
    )
    odom_publisher = driver.create_publisher(
        Odometry, config.odom_topic, qos_profile_sensor_data
    )
    commands: list[tuple[float, float]] = []
    driver.create_subscription(
        TwistStamped,
        config.cmd_vel_topic,
        lambda message: commands.append(
            (message.twist.linear.x, message.twist.angular.z)
        ),
        10,
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(backend.node)
    executor.add_node(driver)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        scan = LaserScan()
        scan.ranges = [1.0, 2.0, 3.0]
        odometry = Odometry()
        odometry.pose.pose.orientation.w = 1.0

        def publish_sensors() -> bool:
            scan_publisher.publish(scan)
            odom_publisher.publish(odometry)
            return backend.is_ready()

        freshness_boundary = time.monotonic()
        wait_until(publish_sensors)
        assert backend.wait_until_ready(not_before_seconds=freshness_boundary)
        observation = backend.read_observation()
        assert observation.healthy
        assert observation.ranges.tolist() == pytest.approx([1.0, 2.0, 3.0])

        backend.execute_action(VelocityCommand(0.1, 0.2, 0.01))
        wait_until(lambda: any(linear > 0.0 for linear, _ in commands))
        wait_until(lambda: commands and commands[-1] == pytest.approx((0.0, 0.0)))

        time.sleep(config.sensor_timeout_seconds + 0.03)
        stale = backend.read_observation()
        assert not stale.healthy
        assert stale.status == 'stale_sensor_data'
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        executor.remove_node(backend.node)
        executor.remove_node(driver)
        backend.close()
        driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

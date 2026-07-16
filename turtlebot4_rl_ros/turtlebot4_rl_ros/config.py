"""ROS adapter configuration."""

from dataclasses import dataclass


@dataclass(frozen=True)
class RosRobotConfig:
    """Topic, frame, and watchdog configuration."""

    scan_topic: str = '/scan'
    odom_topic: str = '/odom'
    cmd_vel_topic: str = '/cmd_vel'
    base_frame: str = 'base_link'
    odom_frame: str = 'odom'
    sensor_timeout_seconds: float = 0.5
    command_timeout_seconds: float = 0.3
    readiness_timeout_seconds: float = 5.0
    command_publish_hz: float = 20.0
    use_sim_time: bool = False
    allow_zero_message_timestamps: bool = True
    future_timestamp_tolerance_seconds: float = 0.1
    clock_jump_tolerance_seconds: float = 0.1

    def __post_init__(self) -> None:
        if (
            self.sensor_timeout_seconds <= 0
            or self.command_timeout_seconds <= 0
            or self.readiness_timeout_seconds <= 0
            or self.command_publish_hz <= 0
            or self.future_timestamp_tolerance_seconds < 0
            or self.clock_jump_tolerance_seconds < 0
        ):
            raise ValueError('timeouts, timestamp tolerances, and command rate must be valid')
        if not all((self.scan_topic, self.odom_topic, self.cmd_vel_topic)):
            raise ValueError('topic names must not be empty')

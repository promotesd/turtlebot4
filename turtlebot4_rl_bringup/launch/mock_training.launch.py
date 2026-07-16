"""Launch a deterministic no-simulator DQN smoke training run."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument('episodes', default_value='10'),
            DeclareLaunchArgument('seed', default_value='42'),
            Node(
                package='turtlebot4_rl_bringup',
                executable='train_mock',
                output='screen',
                arguments=[
                    '--episodes',
                    LaunchConfiguration('episodes'),
                    '--seed',
                    LaunchConfiguration('seed'),
                ],
            ),
        ]
    )

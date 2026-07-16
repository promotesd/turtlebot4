"""Train against an already running TurtleBot 4 Gazebo Harmonic simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    environment_config = PathJoinSubstitution(
        [FindPackageShare('turtlebot4_rl_bringup'), 'config', 'navigation.yaml']
    )
    dqn_config = PathJoinSubstitution(
        [FindPackageShare('turtlebot4_dqn'), 'config', 'dqn.yaml']
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('episodes', default_value='10'),
            DeclareLaunchArgument('seed', default_value='42'),
            DeclareLaunchArgument('checkpoint', default_value=''),
            Node(
                package='turtlebot4_rl_bringup',
                executable='train_rl',
                output='screen',
                arguments=[
                    '--mode',
                    'train',
                    '--episodes',
                    LaunchConfiguration('episodes'),
                    '--seed',
                    LaunchConfiguration('seed'),
                    '--checkpoint',
                    LaunchConfiguration('checkpoint'),
                    '--environment-config',
                    environment_config,
                    '--dqn-config',
                    dqn_config,
                ],
            ),
        ]
    )

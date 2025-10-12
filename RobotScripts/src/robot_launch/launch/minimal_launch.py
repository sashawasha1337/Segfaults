from Segfaults.RobotScripts.src.robot_launch.launch.full_launch import LaunchDescription
from launch_ros.actions import Node
from Segfaults.RobotScripts.src.robot_launch.launch.full_launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    verbosity_arg = DeclareLaunchArgument(
        'verbosity',
        default_value='INFO',
        description = 'Logging verbosity level (DEBUG, INFO, WARN, ERROR, FATAL)',
        choices = ['DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL']
    )

    verbosity = LaunchConfiguration('verbosity')



    return LaunchDescription([
        verbosity_arg,

        Node(
            package='camera',
            executable='camera_node',
            prefix='libcamerify',
            output='screen',
            respawn=True,
            respawn_delay=2,
            arguments=['--ros-args', '--log-level', verbosity]
        ),
        Node(
            package='motor_control',
            executable='motor_node',
            output='screen',
            respawn=True,
            respawn_delay=2,
            arguments=['--ros-args', '--log-level', verbosity]
            ),
        Node(
            package='networking',
            executable='networking_node_script',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
    ])

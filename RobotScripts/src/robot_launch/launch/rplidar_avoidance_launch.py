# launch test for rplidar avoidance node only
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    verbosity_arg = DeclareLaunchArgument(
        'verbosity',
        default_value='INFO',
        description='Logging verbosity level (DEBUG, INFO, WARN, ERROR, FATAL)'
    )

    verbosity = LaunchConfiguration('verbosity')

    return LaunchDescription([
        verbosity_arg,

        Node(
            package='camera', 
            node_executable='rplidar_avoidance',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
    ])

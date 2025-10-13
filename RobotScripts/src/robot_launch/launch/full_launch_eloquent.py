#test launch file for eloquent, based off of full_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    verbosity_arg = DeclareLaunchArgument(
        'verbosity',
        default_value='INFO',
        description='Logging verbosity level (DEBUG, INFO, WARN, ERROR, FATAL)'
    )

    system_arg = DeclareLaunchArgument(
        'system',
        default_value='nano',
        description='Robot system type (nano, pi, dev)'
    )

    verbosity = LaunchConfiguration('verbosity')
    system = LaunchConfiguration('system')

    # Conditional camera prefix for Raspberry Pi
    camera_prefix = PythonExpression([
        "'libcamerify' if '", system, "' == 'pi' else ''"
    ])

    return LaunchDescription([
        verbosity_arg,
        system_arg,

        Node(
            package='camera',
            node_executable='camera_node',
            prefix=camera_prefix,
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
        Node(
            package='camera',
            node_executable='yolo_node',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
        Node(
            package='camera',
            node_executable='rplidar_avoidance',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
        Node(
            package='navigation',
            node_executable='navigation_node',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
        Node(
            package='navigation',
            node_executable='gps_node',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
        Node(
            package='navigation',
            node_executable='geofence_node',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
        Node(
            package='motor_control',
            node_executable='motor_node',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
        Node(
            package='motor_control',
            node_executable='battery_node',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
        Node(
            package='networking',
            node_executable='networking_node_script',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
        Node(
            package='networking',
            node_executable='event_compiler',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
    ])

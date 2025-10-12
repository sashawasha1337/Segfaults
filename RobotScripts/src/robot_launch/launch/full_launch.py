from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    verbosity_arg = DeclareLaunchArgument(
        'verbosity',
        default_value='INFO',
        description = 'Logging verbosity level (DEBUG, INFO, WARN, ERROR, FATAL)',
        choices = ['DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL']
    )

    system_arg = DeclareLaunchArgument(
        'system',
        default_value='nano',
        description = 'Robot system type (nano, pi, dev)',
        choices = ['nano', 'pi', 'dev']
    )

    verbosity = LaunchConfiguration('verbosity')
    system = LaunchConfiguration('system')
 
    # Configure camera prefix for raspberry pi 5
    camera_prefix = PythonExpression([
    "'libcamerify' if '", system, "' == 'pi' else ''"
    ])
    

    return LaunchDescription([
        verbosity_arg,

        Node(
            package='camera',
            executable='camera_node',
            prefix=camera_prefix,
            output='screen',
            respawn=True,
            respawn_delay=10,
            arguments=['--ros-args', '--log-level', verbosity]
        ),
        Node(
            package='camera',
            executable='yolo_node',
            output='screen',
            respawn=True,
            respawn_delay=10,
            arguments=['--ros-args', '--log-level', verbosity]
        ),
        Node(
            package='camera',
            executable='rplidar_avoidance',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
        Node(
            package='navigation',
            executable='navigation_node',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
        Node(
            package='navigation',
            executable='gps_node',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
        Node(
            package='navigation',
            executable='geofence_node',
            output='screen',
            respawn=True,
            respawn_delay=2,
            arguments=['--ros-args', '--log-level', verbosity]
        ),
        Node(
            package='navigation',
            executable='obstacle_avoidance',
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
            package='motor_control',
            executable='battery_node',
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
        Node(
            package='networking',
            executable='event_compiler',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
    ])

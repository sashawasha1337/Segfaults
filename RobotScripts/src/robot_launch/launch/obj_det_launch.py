from launch_ros.actions import Node
from launch import LaunchDescription
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
        system_arg,

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
            executable='simple_image_event_test',
            output='screen',
            respawn=True,
            respawn_delay=10,
            arguments=['--ros-args', '--log-level', verbosity]
        )

    ])

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

    motor_executable = PythonExpression([
        "'rosmaster_motor_node' if '", system, "' == 'nano' else 'motor_node'"
    ])


    return LaunchDescription([
        verbosity_arg,
        system_arg,

        Node(
            package='camera',
            node_executable='firebase_listener',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
#        Node(
#            package='networking',
#            node_executable='firestore_gps_listener',
#            output='screen',
#            arguments=['--ros-args', '--log-level', verbosity]
#        ),
        Node(
            package='networking',
            node_executable='networking_node_script',
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # --- Launch arguments ---
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

    # --- Launch configurations ---
    verbosity = LaunchConfiguration('verbosity')
    system = LaunchConfiguration('system')

    # Conditional camera prefix for Raspberry Pi
    camera_prefix = PythonExpression([
        "'libcamerify' if '", system, "' == 'pi' else ''"
    ])

    # --- LaunchDescription ---
    return LaunchDescription([
        verbosity_arg,
        system_arg,

        # CAMERA NODE
        Node(
            package='camera',
            node_executable='camera_node',
            prefix=camera_prefix,
            output='screen',
            arguments=['--ros-args', '--log-level', verbosity]
        ),

        # YOLO NODE WITH PARAMETERS
        Node(
            package='camera',
            node_executable='yolo_node',
            output='screen',
            parameters=[
                {
                    'image_topic': '/camera/image_raw',
                    'weights': '/Model/fixedbest.pt',
                    'conf': 0.35,
                    'device': 'cuda:0',
                    'img_size': 640,
                    'max_age': 30,
                    'n_init': 3,
                    'max_iou_dist': 0.7,
                    'embedder': 'mobilenet',
                    'half': True,
                }
            ],
            arguments=['--ros-args', '--log-level', verbosity]
        ),
    ])
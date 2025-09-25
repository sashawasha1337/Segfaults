from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera',
            executable='camera_node_script',
            prefix='libcamerify',
            output='screen',
        ),
        Node(
            package='networking',
            executable='networking_node_script',
            output='screen'
        ),
    ])

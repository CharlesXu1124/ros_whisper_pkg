from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    whisper_client_node = Node(
        package='whisper_client_cpp',
        executable='whisper_client_node_exe',
        output='screen'
    )

    return LaunchDescription([
        whisper_client_node
    ])

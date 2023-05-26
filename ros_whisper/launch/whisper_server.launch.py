from launch import LaunchDescription
from launch_ros.actions import Node


# launch script for launching ros2 nodes
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_whisper',
            executable='ros_whisper_node.py',
            output='screen'
        ),
    ])
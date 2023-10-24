from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can2tl',
            executable='can2tl_node',
        ),
    ])
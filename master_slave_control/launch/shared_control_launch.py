# master_slave_control_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='master_slave_control',
            executable='input_mapping.py',
            output='screen',
            name='input_mapping_node'
        ),
    ])


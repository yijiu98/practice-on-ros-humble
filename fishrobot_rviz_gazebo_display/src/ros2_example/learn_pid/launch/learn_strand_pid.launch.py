from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='learn_pid',
            executable='learn_strand_pid_node',
            name='learn_strand_pid_node',
        )
        
    ])

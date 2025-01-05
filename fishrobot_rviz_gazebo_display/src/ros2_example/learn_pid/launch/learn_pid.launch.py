from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='learn_pid',
            executable='learn_pid_node',
            name='learn_pid_node',
        ),
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='rqt_plot',
            arguments=['/feedback/data'],
        )
        
    ])

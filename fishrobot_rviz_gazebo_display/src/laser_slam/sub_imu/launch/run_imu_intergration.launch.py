from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node  # 正确导入 Node

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Starting imu integration node..."),  # 可选的日志消息
        Node(
            package='slam-yijiu98',
            executable='run_imu_intergration',  # 可执行文件的名称
            name='imu_integration_node',
            output='screen',
            parameters=[{'some_param': 'value'}],  # 如果有参数，可以在这里设置
            # remappings=[('/imu', '/imu_data')]  # 如果需要重映射话题
        )
    ])

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取 fishbot_navigation2 包的共享目录
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    # 获取 nav2_bringup 包的共享目录
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # 指定 RViz 配置文件路径
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # 创建启动配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')  # 是否使用模拟时间
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(fishbot_navigation2_dir, 'maps', 'room.yaml'))  # 指定地图文件
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_test_controller.yaml'))  # 指定参数文件

    return launch.LaunchDescription([
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',  # 参数名称
            default_value=use_sim_time,  # 默认值
            description='使用模拟 (Gazebo) 时钟如果设置为 true'  # 参数描述
        ),
        launch.actions.DeclareLaunchArgument(
            'map',  # 参数名称
            default_value=map_yaml_path,  # 默认值为地图路径
            description='地图文件的完整路径'  # 参数描述
        ),
        launch.actions.DeclareLaunchArgument(
            'params_file',  # 参数名称
            default_value=nav2_param_path,  # 默认值为参数文件路径
            description='参数文件的完整路径'  # 参数描述
        ),

        # 包含另一个启动文件
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']  # 指定 bringup_launch.py 的路径
            ),
            # 使用自己声明的参数替换原参数
            launch_arguments={
                'map': map_yaml_path,  # 地图路径
                'use_sim_time': use_sim_time,  # 模拟时间
                'params_file': nav2_param_path  # 参数文件
            }.items(),
        ),

        # 启动 RViz2 节点
        launch_ros.actions.Node(
            package='rviz2',  # RViz2 所在包
            executable='rviz2',  # 执行文件
            name='rviz2',  # 节点名称
            arguments=['-d', rviz_config_dir],  # 指定配置文件
            parameters=[{'use_sim_time': use_sim_time}],  # 配置模拟时间参数
            output='screen'  # 输出到屏幕
        ),
    ])

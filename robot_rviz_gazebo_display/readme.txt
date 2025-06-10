cartographer下载编译:
https://github.com/ros2/cartographer_ros
https://github.com/ros2/cartographer
git clone https://ghproxy.com/https://github.com/ros2/cartographer.git -b ros2
git clone https://ghproxy.com/https://github.com/ros2/cartographer_ros.git -b ros2
colcon build --packages-up-to cartographer_ros
查看安装成功
ros2 pkg list | grep cartographer
cartographer_ros
cartographer_ros_msgs
查看cartographer安装的目录
ros2 pkg prefix cartographer_ros


nav2下载编译：
git clone git@github.com:ros-navigation/navigation2.git -b humble
src同级目录打开终端
wget http://fishros.com/install -O fishros && . fishros
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --packages-up-to navigation2
colcon build --packages-up-to nav2_bringup


依赖：
sudo apt install ros-humble-joint-state-broadcaster ros-humble-diff-drive-controller ros-humble-ros2-control

查看仿真机器人命令：
ros2 launch fishbot_description display_robot.launch.py

导航运行命令：
ros2 launch fishbot_navigation2 navigation2.launch.py
ros2 launch fishbot_description gazebo_sim.launch.py

cartographer建图命令：
ros2 launch fishbot_description gazebo_sim.launch.py
ros2 launch fishbot_cartographer cartographer.launch.py
遥控包
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
安装地图包
sudo apt install ros-humble-nav2-map-server
保存地图
ros2 run nav2_map_server map_saver_cli -f path/map_name
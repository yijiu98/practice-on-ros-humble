ros2 pkg create fishbot_navigation2 --dependencies nav2_bringup
src/navigation2/nav2_bringup/bringup/params/nav2_params.yaml的内容复制粘贴到当前的config文件中

colcon build --packages-select fishbot_navigation2

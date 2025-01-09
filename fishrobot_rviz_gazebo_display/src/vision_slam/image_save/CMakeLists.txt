cmake_minimum_required(VERSION 3.8)
project(image_save)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_compile_options(-std=c++17)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveDepthData.srv"
  DEPENDENCIES geometry_msgs
)

# 确保接口生成的头文件路径包含在构建目标中
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# 添加源文件
add_executable(image_save_server src/image_save_server.cpp)
ament_target_dependencies(image_save_server
  rclcpp
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
  OpenCV
)

# 添加接口生成依赖,rosidl_target_interfaces 是 ROS 2 CMake 中的一种工具，用于将生成的接口（例如 .srv、.msg 文件）与特定目标（如可执行文件或库）关联
rosidl_target_interfaces(image_save_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装目标
install(TARGETS
  image_save_server  
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

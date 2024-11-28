cmake_minimum_required(VERSION 3.8)
project(vision_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( "/usr/include/eigen3/" )
# pcl 
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

add_executable(vision_slam src/joint_map.cpp)

target_link_libraries( vision_slam ${OpenCV_LIBS} ${PCL_LIBRARIES} )
# 安装 launch 文件
install(TARGETS vision_slam
  DESTINATION lib/${PROJECT_NAME}  # 确保安装路径正确
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
ament_package()

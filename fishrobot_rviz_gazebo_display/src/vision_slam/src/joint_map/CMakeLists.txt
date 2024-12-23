project(joint_map)
add_executable(joint_map joint_map.cpp)
target_link_libraries( joint_map ${OpenCV_LIBS} ${PCL_LIBRARIES} )


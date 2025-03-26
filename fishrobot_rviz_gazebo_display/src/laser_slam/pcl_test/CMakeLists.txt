cmake_minimum_required(VERSION 3.10)
project(pcl_test)

find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)
find_package(Glog REQUIRED)
find_package(GFlags REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgcodecs highgui)  # 明确指定模块[1,3](@ref)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${GFLAGS_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(point_cloud_load_and_vis point_cloud_load_and_vis.cpp)

target_link_libraries(point_cloud_load_and_vis
    ${PCL_LIBRARIES}
    # ${GLOG_LIBRARIES}
    # ${GFLAGS_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libglog.so
    /usr/lib/x86_64-linux-gnu/libgflags.so
    pthread
)

add_executable(pcb_to_bird_eye pcb_to_bird_eye.cpp)
target_link_libraries(pcb_to_bird_eye 
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  # ${GLOG_LIBRARIES}
  # ${GFLAGS_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libglog.so
  /usr/lib/x86_64-linux-gnu/libgflags.so
  pthread
)  # 自动链接所有已找到的模块



install(TARGETS point_cloud_load_and_vis
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS pcb_to_bird_eye
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
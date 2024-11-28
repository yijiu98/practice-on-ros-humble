服务调用的命令
ros2 service call /save_depth_data image_save/srv/SaveDepthData "{save_path: '/home/yijiu98/yijiu98/ros/ros2humblepactices/fishrobot_rviz_gazebo_display/cash'}"

cv_ptr->image.type() 返回的是 OpenCV 的图像类型，它用来描述图像的通道数和每个像素的存储深度。常见的类型包括：

CV_8U：8位无符号整数，单通道
CV_16U：16位无符号整数，单通道
CV_32F：32位浮点数，单通道
CV_32F 类型表示图像的深度值是以 32 位浮点数表示的，而不是常见的 16 位无符号整数（CV_16U）。这是某些深度摄像头（例如使用 OpenNI 或 RealSense SDK 的设备）输出的常见格式。
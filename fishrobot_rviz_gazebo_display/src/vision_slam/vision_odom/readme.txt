sudo apt-get install libeigen3-dev
sudo apt-get install libpcl-all
针对joint_map程序，需要获取当前位置中机器人在世界坐标系的xyz和四元数，可以通过rviz中tf中的odom显示，或者获取到。


image_save:
提供保存图像和深度图像的服务

laser_slam:
雷达激光的slam测试

use_eigen：
使用eigen的库测试

useGeometry:
使用geometry库的测试,旋转，欧拉角，四元数
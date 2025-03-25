cd ~/ros/src
mkdir laoerExample
cd laoerExample
ros2 pkg create --build-type ament_cmake cPubSub
colcon build --packages-select cPubSub

ros2 run cPubSub talker
ros2 run cPubSub listener
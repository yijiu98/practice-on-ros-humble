#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp" // 包含 IMU 消息类型

class ImuSubscriberNode : public rclcpp::Node
{
public:
    ImuSubscriberNode() : Node("imu_intergration_node")
    {
        // 创建订阅者
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", // 订阅的话题名
            100,    // 队列大小
            std::bind(&ImuSubscriberNode::imuCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "IMU Subscriber Node started.");
    }

private:
    // 回调函数，用于处理接收到的 IMU 数据
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 打印 IMU 数据
        RCLCPP_INFO(this->get_logger(), "IMU Orientation: [x: %.2f, y: %.2f, z: %.2f, w: %.2f]",
                    msg->orientation.x,
                    msg->orientation.y,
                    msg->orientation.z,
                    msg->orientation.w);

        RCLCPP_INFO(this->get_logger(), "Angular Velocity: [x: %.2f, y: %.2f, z: %.2f]",
                    msg->angular_velocity.x,
                    msg->angular_velocity.y,
                    msg->angular_velocity.z);

        RCLCPP_INFO(this->get_logger(), "Linear Acceleration: [x: %.2f, y: %.2f, z: %.2f]",
                    msg->linear_acceleration.x,
                    msg->linear_acceleration.y,
                    msg->linear_acceleration.z);
    }

    // 声明订阅者
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

int main(int argc, char **argv)
{
    // 初始化 ROS 2 节点
    rclcpp::init(argc, argv);
    auto imu_node = std::make_shared<ImuSubscriberNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(imu_node);
     executor.spin();
    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}

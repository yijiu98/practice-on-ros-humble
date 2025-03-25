#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    /* 公共构造函数将节点命名为minimal_publisher，并将count_初始化为0 */
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      /* 初始化发布者publisher_ ，使用String消息类型、主题名称topic和在发生备份时限制消息所需的队列大小10 */
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

      /* 初始化timer_，设置timer_callback函数每500ms执行一次 */
      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    /* 定义定时器回调函数 */
    void timer_callback()
    {
      /* 打印并发布字符串信息 */
      auto message = std_msgs::msg::String();
      message.data = "Hello, world: " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }

    /* 计时器、发布者和计数器字段的声明 */
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  /* 初始化ROS2 */
  rclcpp::init(argc, argv);

  /* 运行节点MinimalPublisher */
  rclcpp::spin(std::make_shared<MinimalPublisher>());

  /* 退出ROS2 */
  rclcpp::shutdown();
  return 0;
}

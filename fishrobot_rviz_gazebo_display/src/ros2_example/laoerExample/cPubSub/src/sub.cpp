#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/* 占位符，代替回调函数中的第一个参数 */
using std::placeholders::_1;


/* 继承rclcpp:: node创建节点类MinimalSubscriber */
class MinimalSubscriber : public rclcpp::Node
{
  public:
    /* 公共构造函数将节点命名为minimal_subscriber */
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      /* 初始化订阅者subscription_  ，使用String消息类型、主题名称topic和在发生备份时限制消息所需的队列大小10，
         订阅话题回调函数topic_callback */
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    /* 定义订阅话题回调函数 */
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      /* 打印话题消息的字符串信息 */
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }

    /* 订阅者字段的声明 */
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  /* 初始化ROS2 */
  rclcpp::init(argc, argv);

  /* 运行节点MinimalSubscriber*/
  rclcpp::spin(std::make_shared<MinimalSubscriber>());

  /* 退出ROS2 */
  rclcpp::shutdown();
  return 0;
}

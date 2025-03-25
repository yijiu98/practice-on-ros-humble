#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

/* 求和函数 */
void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
  /* 从请求中获取两个整数，并将相加的结果赋值给响应 */
  response->sum = request->a + request->b;

  /* RCLCPP_INFO：ROS2自带的log输出，分等级，带颜色，输出格式与printf相同，需要标明数据的类型。*/
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld", request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  /* 初始化ROS2 */
  rclcpp::init(argc, argv);

  /* 定义服务端节点add_two_ints_server */
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  /* 创建服务名为add_two_ints，服务函数为add的service服务端 */
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  /* 通知准备就绪 */
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  /* 运行节点 */
  rclcpp::spin(node);

  /* 退出ROS2 */
  rclcpp::shutdown();
}

#include "rclcpp/rclcpp.hpp"

class Nav2Node : public rclcpp::Node
{
public:
  Nav2Node() : Node("nav2_node")
  {
    RCLCPP_INFO(this->get_logger(), "Nav2 Node has been started.");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2Node>());
  rclcpp::shutdown();
  return 0;
}
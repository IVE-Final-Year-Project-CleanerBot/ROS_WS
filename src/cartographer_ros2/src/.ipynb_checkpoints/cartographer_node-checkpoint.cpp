#include "rclcpp/rclcpp.hpp"

class CartographerNode : public rclcpp::Node
{
public:
  CartographerNode() : Node("cartographer_node")
  {
    RCLCPP_INFO(this->get_logger(), "Cartographer Node has been started.");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartographerNode>());
  rclcpp::shutdown();
  return 0;
}
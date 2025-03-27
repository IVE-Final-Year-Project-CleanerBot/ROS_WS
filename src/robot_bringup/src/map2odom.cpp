#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

class StaticTransformPublisher : public rclcpp::Node
{
public:
    StaticTransformPublisher()
        : Node("static_transform_publisher")
    {
        // 创建静态 TF 广播器
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // 定义静态变换
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->get_clock()->now();
        static_transform.header.frame_id = "map";  // 父坐标系
        static_transform.child_frame_id = "odom";  // 子坐标系

        // 设置平移和旋转
        static_transform.transform.translation.x = 0.0;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;
        static_transform.transform.rotation.x = 0.0;
        static_transform.transform.rotation.y = 0.0;
        static_transform.transform.rotation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;

        // 发布静态变换
        static_broadcaster_->sendTransform(static_transform);
        RCLCPP_INFO(this->get_logger(), "Static transform published: map -> odom");
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTransformPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
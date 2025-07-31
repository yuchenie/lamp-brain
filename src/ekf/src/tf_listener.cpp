#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <memory>
#include <iostream>

class TFListenerNode : public rclcpp::Node{
public:
    TFListenerNode() : Node("tf_listener") {
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>("tf", 1, std::bind(&TFListenerNode::tf_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;

    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        for (const auto & transform_stamped : msg->transforms) {
            const auto & q = transform_stamped.transform.rotation;
            tf2::Quaternion quat(q.x, q.y, q.z, q.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            RCLCPP_INFO(this->get_logger(),
                  "Frame: %s -> %s | Roll: %.3f, Pitch: %.3f, Yaw: %.3f (rad)",
                  transform_stamped.header.frame_id.c_str(),
                  transform_stamped.child_frame_id.c_str(),
                  roll, pitch, yaw);
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListenerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
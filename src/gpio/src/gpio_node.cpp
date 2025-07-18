#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include <cmath>
#include <vector>
#include <pigpiod_if2.h>

class GPIONode : public rclcpp::Node{
public: 
    GPIONode() : Node("gpio_node") {
        pi_ = pigpio_start(NULL, NULL);
        if (pi_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to pigpiod");
            rclcpp::shutdown();
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "pigpio initialized");
        } 

        set_mode(pi_, 17, PI_OUTPUT);

        rc_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>("rc_msg", 1, std::bind(&GPIONode::rc_callback, this, std::placeholders::_1));
    }

    ~GPIONode() {
        if (pi_ >= 0) {
            pigpio_stop(pi_);
        }
    }

private:
    int pi_{-1};
    float velocity_ = 0;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr rc_sub_;

    void rc_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        velocity_ = msg->speed;
        int pwm = static_cast<int>(std::abs(velocity_) * 255);
        set_PWM_dutycycle(pi_, 17, pwm);
        RCLCPP_INFO(this->get_logger(), "Velocity: %f", velocity_);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPIONode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
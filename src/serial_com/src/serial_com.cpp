#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include <string>
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include <iostream>

class SerialNode : public rclcpp::Node {
    public:
        SerialNode(): Node("serial_node") {
            this->declare_parameter<std::string>("port", "/dev/ttyS0");

            // Initialize serial port
            try {
                serial_port_.setPort(this->get_parameter("port").as_string());
                serial_port_.setBaudrate(9600);
                serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
                serial_port_.setTimeout(timeout);
                serial_port_.open();

                if (serial_port_.isOpen()) {
                    RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully.");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Failed to open serial port.");
                }
            } catch (const serial::IOException& e) {
                RCLCPP_ERROR(this->get_logger(), "Serial port error: %s", e.what());
            }

            imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
            mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);

            rc_movement_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>("rc_msg", 1, std::bind(&SerialNode::rcMovementCallback, this, std::placeholders::_1));
            serial_pub_ = this->create_publisher<std_msgs::msg::String>("serial_msg", 1);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SerialNode::timer_callback, this));
        }

    private:
        serial::Serial serial_port_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Data to publish to arduino
        float steering_angle_;
        float velocity_ = 0;

        // Parsed serial data from arduino
        int32_t reported_timestamp;
        float reported_steering_angle;
        std::string serial_buffer_;

        rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr rc_movement_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_pub_;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;

        float mag_x = 0.0;
        float mag_y = 0.0;
        float mag_z = 0.0;
        float acc_x = 0.0;
        float acc_y = 0.0;
        float acc_z = 0.0;
        float gyro_x = 0.0;
        float gyro_y = 0.0;
        float gyro_z = 0.0;
        
        void rcMovementCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
            steering_angle_ = msg->steering_angle;
            velocity_ = msg->speed;
        }

        void timer_callback() {
            if (!serial_port_.isOpen()) {
                try {
                    serial_port_.open();
                    RCLCPP_WARN(this->get_logger(), "Serial port re-opened.");
                } catch (const serial::IOException& e) {
                    RCLCPP_ERROR(this->get_logger(), "Reopen failed: %s", e.what());
                }
                return;
            }

            if (serial_port_.isOpen()) {

                // write rc movement message
                char buffer[128];

                float steer = steering_angle_;
                float velocity = velocity_;

                snprintf(buffer, sizeof(buffer), "(%f,%f)", steer, velocity);

                if (serial_port_.write(reinterpret_cast<const uint8_t*>(buffer), strlen(buffer))) {
                    // RCLCPP_INFO(this->get_logger(), "Sent: (%f,%f,%f)", steer, brake, throttle);
                    // RCLCPP_INFO(this->get_logger(), "%s", buffer);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Serial write failed");
                } 

                // read from serial port
                try {
                    size_t bytes_available = serial_port_.available();
                    if (bytes_available > 0) {
                        std::string data = serial_port_.read(bytes_available);
                        serial_buffer_ += data;

                        // Extract complete lines
                        size_t pos = 0;
                        while ((pos = serial_buffer_.find('\n')) != std::string::npos) {
                            std::string line = serial_buffer_.substr(0, pos);
                            serial_buffer_.erase(0, pos + 1);

                            if (!line.empty()) {
                                publishSerial(line);
                            }
                        }
                    }
                } catch (const serial::IOException& e) {
                    RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
                }
                serial_port_.flushInput();
            } else {
                RCLCPP_INFO(this->get_logger(), "Serial port closed.");
            }
        }

        // Function to publish sensor data
        void publishSerial(const std::string & msg) {
            auto message = std_msgs::msg::String();
            auto imu_msg = sensor_msgs::msg::Imu{};
            auto mag_msg = sensor_msgs::msg::MagneticField{};
            message.data = msg;

            auto current_time = this->now().nanoseconds();
            double current_time_seconds = current_time / 1e9;

            sscanf(msg.c_str(),
                "(%f,%f,%f) (%f,%f,%f) (%f,%f,%f)",
                &mag_x, &mag_y, &mag_z,
                &acc_x, &acc_y, &acc_z,
                &gyro_x, &gyro_y, &gyro_z
            );

            // need to convert units here, otherwise resolution too low
            mag_x *= 1e-6;
            mag_y *= 1e-6;
            mag_z *= 1e-6;

            // RCLCPP_INFO(this->get_logger(), "(%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f)", 
            //     mag_x, mag_y, mag_z,
            //     acc_x, acc_y, acc_z,
            //     gyro_x, gyro_y, gyro_z
            // );

            imu_msg.linear_acceleration.x = acc_x;
            imu_msg.linear_acceleration.y = acc_y;
            imu_msg.linear_acceleration.z = acc_z;
            imu_msg.linear_acceleration_covariance = {
                0.1, 0, 0,
                0, 0.1, 0,
                0, 0, 0.1
            };

            imu_msg.angular_velocity.x = gyro_x;
            imu_msg.angular_velocity.y = gyro_y;
            imu_msg.angular_velocity.z = gyro_z;
            imu_msg.angular_velocity_covariance = {
                0.01, 0, 0,
                0, 0.01, 0,
                0, 0, 0.01
            };

            imu_msg.orientation_covariance[0] = -1;
            imu_msg.header.stamp.sec = current_time_seconds;
            imu_msg.header.stamp.nanosec = current_time;

            mag_msg.magnetic_field.x = mag_x; // in Tesla
            mag_msg.magnetic_field.y = mag_y;
            mag_msg.magnetic_field.z = mag_z;

            mag_msg.magnetic_field_covariance = {
                0.0001, 0, 0,
                0, 0.0001, 0,
                0, 0, 0.0001
            };

            mag_msg.header.stamp.sec = current_time_seconds;
            mag_msg.header.stamp.nanosec = current_time;

            imu_msg.header.frame_id = "base_link";
            mag_msg.header.frame_id = "base_link";
            
            imu_pub_->publish(imu_msg);
            mag_pub_->publish(mag_msg);
            serial_pub_->publish(message);
        }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
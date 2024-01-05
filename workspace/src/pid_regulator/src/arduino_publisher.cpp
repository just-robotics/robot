#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"


using std::placeholders::_1;


class ArduinoPublisher : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr subscription_;

    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr target_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr u_publisher_;

public:
    ArduinoPublisher();

private:
    void callback(const std_msgs::msg::Int64MultiArray & msg) const; 
};


ArduinoPublisher::ArduinoPublisher() : Node("serial_publisher") {
    subscription_ = this->create_subscription<std_msgs::msg::Int64MultiArray>("/serial/pub", 10, std::bind(&ArduinoPublisher::callback, this, _1));
#if 0
    pose_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/arduino/pose", 10);
    target_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/arduino/target", 10);
    u_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/arduino/u", 10);
#endif
}


void ArduinoPublisher::callback(const std_msgs::msg::Int64MultiArray & msg) const {
#if 0
    auto pose_msg = std_msgs::msg::Int64();
    auto target_msg = std_msgs::msg::Int64();
    auto u_msg = std_msgs::msg::Int64();

    pose_msg.data = pose;
    target_msg.data = target;
    u_msg.data = u;

    pose_publisher_->publish(pose_msg);
    target_publisher_->publish(target_msg);
    u_publisher_->publish(u_msg);
#endif

    for (size_t i = 0; i < msg.data.size(); i++) {
        std::cout << msg.data[i] << " ";
    }
    std::cout << std::endl;
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoPublisher>());
    rclcpp::shutdown();

    return 0;
}

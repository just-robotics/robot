#ifndef SERIAL_ROS_SERIAL_HPP
#define SERIAL_ROS_SERIAL_HPP


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "connect.hpp"


class RosSerial : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;

public:
    RosSerial();

private:
    std_msgs::msg::Int64MultiArray createRosMsg(Msg* serial_msg, std::vector<size_t> data_idx);
    Msg createSerialMsg(std_msgs::msg::Float32MultiArray ros_msg, std::vector<size_t> data_idx);

    void timerCallback();
    void subscriptionCallback(const std_msgs::msg::Float32MultiArray & msg);     
};


#endif // SERIAL_ROS_SERIAL_HPP

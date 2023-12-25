#ifndef SERIAL_SERIAL_PUBLISHER_HPP
#define SERIAL_SERIAL_PUBLISHER_HPP


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"

#include "connect.hpp"


class RosSerial : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr publisher_;

public:
    RosSerial();

private:
    std_msgs::msg::Int64MultiArray createRosMsg(Msg* msg, std::vector<size_t> data_idx);

    void timer_callback();
    void subscription_callback();    
};


RosSerial::RosSerial() : Node("serial_publisher") {
    using namespace std::chrono_literals;
    publisher_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/serial/pub", 10);
    timer_ = this->create_wall_timer(
    100ms, std::bind(&RosSerial::timer_callback, this));
}


std_msgs::msg::Int64MultiArray RosSerial::createRosMsg(Msg* msg, std::vector<size_t> data_idx) {
    auto message = std_msgs::msg::Int64MultiArray();

    for (size_t i = 0; i < data_idx.size(); i++) {
        int64_t data = Connect::uint8arr_to_int64(msg->msg() + data_idx[i]);
        message.data.push_back(data);
    }

    return message;
}


void RosSerial::timer_callback() {
    Msg msg = Connect::receiveMessage(POSE_MSG_SIZE);
    if (Connect::checkFeedback()) {
        auto ros_message = createRosMsg(&msg, Msgs::pose_target_idx);
        publisher_->publish(ros_message);

        // int64_t pose = ros_message.data[0];
        // int64_t target = ros_message.data[1];
        // int64_t u = ros_message.data[2];

        // std::cout << pose << " " << target << " " << u << std::endl;
    }
}


void RosSerial::subscription_callback() {

}


#endif // SERIAL_SERIAL_PUBLISHER_HPP

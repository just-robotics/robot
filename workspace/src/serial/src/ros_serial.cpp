#include "../include/serial/ros_serial.hpp"


RosSerial::RosSerial() : Node("ros_serial") {
    using namespace std::chrono_literals;
    using std::placeholders::_1;
    publisher_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/serial/pub", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&RosSerial::timerCallback, this));
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/pid", 10, std::bind(&RosSerial::subscriptionCallback, this, _1));
}


std_msgs::msg::Int64MultiArray RosSerial::createRosMsg(Msg* serial_msg, std::vector<size_t> data_idx) {
    auto ros_msg = std_msgs::msg::Int64MultiArray();

    for (size_t i = 0; i < data_idx.size(); i++) {
        int64_t data = Connect::uint8arr_to_int64(serial_msg->msg() + data_idx[i]);
        ros_msg.data.push_back(data);
    }

    return ros_msg;
}


void RosSerial::timerCallback() {
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


Msg RosSerial::createSerialMsg(std_msgs::msg::Float32MultiArray ros_msg, std::vector<size_t> data_idx) {
    Msg serial_msg(4 + data_idx.size() * 4);

    for (size_t i = 0; i < data_idx.size(); i++) {
        float data = ros_msg.data[i];
        Connect::float_to_uint8arr(data, serial_msg.msg() + data_idx[i]);
    }

    return serial_msg;
}


void RosSerial::subscriptionCallback(const std_msgs::msg::Float32MultiArray & msg) {
    float kp = msg.data[0];
    float kd = msg.data[1];
    float ki = msg.data[2];

    Msg serial_msg = createSerialMsg(msg, Msgs::pid_idx);

    auto n = createRosMsg(&serial_msg, Msgs::pid_idx);

    kp = msg.data[0];
    kd = msg.data[1];
    ki = msg.data[2];

    std::cout << "kp = " << kp << "; kd = " << kd << "; ki = " << ki << ";" << std::endl;

    Connect::sendCommand(&serial_msg);
}

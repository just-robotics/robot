#include "../include/serial/ros_serial.hpp"


RosSerial::RosSerial() : Node("ros_serial") {
    using namespace std::chrono_literals;
    using std::placeholders::_1;
    publisher_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/serial/pub", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&RosSerial::timerCallback, this));
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/pid_regulator/pid", 10, std::bind(&RosSerial::subscriptionCallback, this, _1));
}


std_msgs::msg::Int64MultiArray RosSerial::createRosMsg(Msg* serial_msg, std::vector<size_t> data_idx) {
    auto ros_msg = std_msgs::msg::Int64MultiArray();

    for (size_t i = 0; i < data_idx.size(); i++) {
        int64_t data = serial::uint8arr_to_int64(serial_msg->msg() + data_idx[i]);
        ros_msg.data.push_back(data);
    }

    return ros_msg;
}


void RosSerial::timerCallback() {
    Msg msg = serial::receiveMessage(MsgSizes::POSE);
    if (serial::checkFeedback()) {
        auto ros_message = createRosMsg(&msg, Msgs::pose_idx);
        publisher_->publish(ros_message);
#if 0
        int64_t pose0 = ros_message.data[0];
        int64_t pose1 = ros_message.data[1];
        int64_t pose2 = ros_message.data[2];
        int64_t pose3 = ros_message.data[3];

        std::cout << "POSES: " << pose0 << " " << pose1 << " " << pose2 << " " << pose3 << std::endl;
#endif
    }
}


Msg RosSerial::createSerialMsg(std_msgs::msg::Float32MultiArray ros_msg, std::vector<size_t> data_idx) {
    Msg serial_msg(4 + data_idx.size() * 4);

    for (size_t i = 0; i < data_idx.size(); i++) {
        float data = ros_msg.data[i];
        serial::float_to_uint8arr(data, serial_msg.msg() + data_idx[i]);
    }

    return serial_msg;
}


void RosSerial::subscriptionCallback(const std_msgs::msg::Float32MultiArray & ros_msg) {
    Msg serial_msg = createSerialMsg(ros_msg, Msgs::pid_idx);

    float kp = ros_msg.data[0];
    float kd = ros_msg.data[1];
    float ki = ros_msg.data[2];

    std::cout << "kp = " << kp << "; kd = " << kd << "; ki = " << ki << ";" << std::endl;

    serial_msg.set_task(Tasks::SET_PID);

    serial::sendCommand(&serial_msg);
}

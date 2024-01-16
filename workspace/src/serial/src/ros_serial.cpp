#include "../include/serial/ros_serial.hpp"


RosSerial::RosSerial() : Node("ros_serial") {
    using namespace std::chrono_literals;
    using std::placeholders::_1;
    publisher_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/serial/pub", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&RosSerial::timerCallback, this));
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/pid_regulator/pid", 10, std::bind(&RosSerial::subscriptionCallback, this, _1));
    this->declare_parameter("serial_name", "");
    this->declare_parameter("serial_baudrate", 0);
    std::string serial_name = this->get_parameter("serial_name").as_string();
    size_t baudrate = this->get_parameter("serial_baudrate").as_int();

    //serial_ = new Serial(serial_name, baudrate, "");
    serial_ = new Serial("/dev/ttyACM0", 2'000'000, "");
    if (!serial_->is_opened()) {
        RCLCPP_FATAL(this->get_logger(), "Unable to connect to serial device '%s' with baudrate %ld", serial_->name().c_str(), serial_->baudrate());
        rclcpp::shutdown();
    }
    serial_->connect();
}


RosSerial::~RosSerial() {
    delete serial_;
}


std_msgs::msg::Int64MultiArray RosSerial::createRosMsg(Msg* serial_msg, std::vector<size_t> data_idx) {
    auto ros_msg = std_msgs::msg::Int64MultiArray();

    for (size_t i = 0; i < data_idx.size(); i++) {
        int64_t data = Serial::uint8arr_to_int64(serial_msg->data() + data_idx[i]);
        ros_msg.data.push_back(data);
    }

    return ros_msg;
}


void RosSerial::timerCallback() {
    Msg msg = serial_->receive(msg_sizes::POSE);
    if (serial_->checkFeedback()) {
        auto ros_message = createRosMsg(&msg, serial_msgs::pose_idx);
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
        Serial::float_to_uint8arr(data, serial_msg.data() + data_idx[i]);
    }

    return serial_msg;
}


void RosSerial::subscriptionCallback(const std_msgs::msg::Float32MultiArray & ros_msg) {
    Msg serial_msg = createSerialMsg(ros_msg, serial_msgs::pid_idx);

    float kp = ros_msg.data[0];
    float kd = ros_msg.data[1];
    float ki = ros_msg.data[2];

    std::cout << "kp = " << kp << "; kd = " << kd << "; ki = " << ki << ";" << std::endl;

    serial_msg.setTask(tasks::SET_PID);

    serial_->send(&serial_msg);
}

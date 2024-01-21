#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_msgs/msg/odom.hpp"
#include "robot_msgs/msg/u_int8_vector.hpp"
#include "robot_msgs/msg/float32_vector.hpp"


using std::placeholders::_1;


class SerialBridge : public rclcpp::Node {
private:
    rclcpp::Publisher<robot_msgs::msg::UInt8Vector>::SharedPtr serial_pub_;
    rclcpp::Subscription<robot_msgs::msg::UInt8Vector>::SharedPtr serial_sub_;
    rclcpp::Publisher<robot_msgs::msg::Odom>::SharedPtr odom_pub_;
    rclcpp::Subscription<robot_msgs::msg::Float32Vector>::SharedPtr cmd_vel_sub_;
    size_t pose_num_, pose_size_, vel_num_, vel_size_;

public:
    SerialBridge();

private:
    void serialCallback(const robot_msgs::msg::UInt8Vector& msg) const;
    void cmdVelCallback(const robot_msgs::msg::Float32Vector& msg) const;
};


SerialBridge::SerialBridge() : Node("serial_bridge") {
    this->declare_parameter("pose_num", 0);
    this->declare_parameter("pose_size", 0);
    this->declare_parameter("vel_num", 0);
    this->declare_parameter("vel_size", 0);
    this->declare_parameter("serial_pub", "");
    this->declare_parameter("serial_sub", "");
    this->declare_parameter("odom_pub", "");
    this->declare_parameter("vel_sub", "");

    pose_num_ = this->get_parameter("pose_num").as_int();
    pose_size_ = this->get_parameter("pose_size").as_int();
    vel_num_ = this->get_parameter("vel_num").as_int();
    vel_size_ = this->get_parameter("vel_size").as_int();
    std::string serial_pub_topic = this->get_parameter("serial_pub").as_string();
    std::string serial_sub_topic = this->get_parameter("serial_sub").as_string();
    std::string odom_pub_topic = this->get_parameter("odom_pub").as_string();
    std::string vel_sub_topic = this->get_parameter("vel_sub").as_string();

    RCLCPP_INFO(this->get_logger(), "vel_num: %ld", pose_num_);
    RCLCPP_INFO(this->get_logger(), "vel_size: %ld", pose_size_);
    RCLCPP_INFO(this->get_logger(), "vel_num: %ld", vel_num_);
    RCLCPP_INFO(this->get_logger(), "vel_size: %ld", vel_size_);
    RCLCPP_INFO(this->get_logger(), "serial_pub_topic: '%s'", serial_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "serial_sub_topic: '%s'", serial_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "odom_pub_topic: '%s'", odom_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "vel_sub_topic: '%s'", vel_sub_topic.c_str());

    serial_pub_ = this->create_publisher<robot_msgs::msg::UInt8Vector>(serial_pub_topic, 10);
    odom_pub_ = this->create_publisher<robot_msgs::msg::Odom>(odom_pub_topic, 10);

    serial_sub_ = this->create_subscription<robot_msgs::msg::UInt8Vector>(serial_sub_topic, 10, std::bind(&SerialBridge::serialCallback, this, _1));
    cmd_vel_sub_ = this->create_subscription<robot_msgs::msg::Float32Vector>(vel_sub_topic, 10, std::bind(&SerialBridge::cmdVelCallback, this, _1));
}


void SerialBridge::serialCallback(const robot_msgs::msg::UInt8Vector& msg) const {
    auto odom = robot_msgs::msg::Odom();

    for (size_t i = 0; i < pose_num_; i++) {
        int64_t pose;
        std::memcpy(&pose, msg.data.data() + i * pose_size_, pose_size_);
        odom.poses.push_back(pose);
    }

    for (size_t i = 0; i < pose_num_; i++) {
        float vel;
        std::memcpy(&vel, msg.data.data() + pose_num_ * pose_size_ + i * vel_size_, vel_size_);
        odom.velocities.push_back(vel);
    }

    odom_pub_->publish(odom);
}


void SerialBridge::cmdVelCallback(const robot_msgs::msg::Float32Vector& msg) const {
    auto serial_msg = robot_msgs::msg::UInt8Vector();
    serial_msg.data.resize(vel_num_ * vel_size_);

    for (size_t i = 0; i < msg.data.size(); i++) {
        std::memcpy(serial_msg.data.data() + i * vel_size_, msg.data.data() + i, vel_size_);
    }

    serial_pub_->publish(serial_msg);
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialBridge>()); 
    rclcpp::shutdown();

    return 0;
}

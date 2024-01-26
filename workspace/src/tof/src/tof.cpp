#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include "robot_msgs/msg/u_int8_vector.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


class Tof : public rclcpp::Node {
private:
    rclcpp::Subscription<robot_msgs::msg::UInt8Vector>::SharedPtr serial_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr tof_pub_;

    size_t data_num_, data_size_;
    std::string frame_id_;

public:
    Tof();

private:    
    void callback(const robot_msgs::msg::UInt8Vector& msg);
};


Tof::Tof() : Node("tof") {
    this->declare_parameter("tof_pub", "");
    this->declare_parameter("serial_pub", "");
    this->declare_parameter("data_num", 0);
    this->declare_parameter("data_size", 0);
    this->declare_parameter("frame_id", "");

    std::string tof_pub_topic = this->get_parameter("tof_pub").as_string();
    std::string serial_pub_topic = this->get_parameter("serial_pub").as_string();
    data_num_ = this->get_parameter("data_num").as_int();
    data_size_ = this->get_parameter("data_size").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();

    RCLCPP_INFO(this->get_logger(), "tof_pub_topic: '%s'", tof_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "serial_pub_topic: '%s'", serial_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "data_num: %ld", data_num_);
    RCLCPP_INFO(this->get_logger(), "data_size: %ld", data_size_);
    RCLCPP_INFO(this->get_logger(), "frame_id: '%s'", frame_id_.c_str());

    tof_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(tof_pub_topic, 10);
    serial_sub_ = this->create_subscription<robot_msgs::msg::UInt8Vector>(serial_pub_topic, 10, std::bind(&Tof::callback, this, _1));
}


void Tof::callback(const robot_msgs::msg::UInt8Vector& msg) {
    std::vector<float> data;

    for (size_t i = 0; i < data_num_; i++) {
        float d;
        std::memcpy(&d, msg.data.data() + i * data_size_, data_size_);
        std::cout << d << " ";
        data.push_back(d);
    }
    std::cout << std::endl;

    auto laser_scan = sensor_msgs::msg::LaserScan();

    laser_scan.header.frame_id = frame_id_;
    laser_scan.header.stamp = this->get_clock()->now();

    laser_scan.angle_min = 0;
    laser_scan.angle_max = 3.14;
    laser_scan.angle_increment = 3.14 / 12;

    laser_scan.time_increment = 0.05;
    laser_scan.scan_time = 0.6;

    laser_scan.range_min = 0.04;
    laser_scan.range_max = 4.00;

    laser_scan.ranges = data;

    tof_pub_->publish(laser_scan);
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tof>()); 
    rclcpp::shutdown();

    return 0;
}

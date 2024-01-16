#include <iostream>

#include "../include/serial/serial.hpp"
#include "../include/serial/ros_serial.hpp"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosSerial>());
    rclcpp::shutdown();
    
    return 0;
}

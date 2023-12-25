#include <iostream>

#include "../include/serial/connect.hpp"
#include "../include/serial/serial_publisher.hpp"


int main(int argc, char** argv) {
    if (!Connect::setConnection()) {
        return 1;
    };

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosSerial>());
    rclcpp::shutdown();
    
    return 0;
}

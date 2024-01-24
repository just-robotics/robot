cmake_minimum_required(VERSION 3.8)
project(tof)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)

add_executable(tof src/tof.cpp)
add_executable(ros_serial src/ros_serial.cpp)

ament_target_dependencies(tof rclcpp robot_msgs sensor_msgs)
ament_target_dependencies(ros_serial rclcpp robot_msgs serial)

install(DIRECTORY
    config src
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    tof
    ros_serial
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

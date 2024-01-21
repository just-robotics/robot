cmake_minimum_required(VERSION 3.8)
project(serial_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msgs REQUIRED)

add_executable(drive_controller_bridge
    src/drive_controller_bridge.cpp
)

ament_target_dependencies(drive_controller_bridge rclcpp robot_msgs)

install(DIRECTORY
    src include launch config
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    drive_controller_bridge
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

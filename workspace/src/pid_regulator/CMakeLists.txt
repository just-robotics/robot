cmake_minimum_required(VERSION 3.8)
project(pid_regulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(arduino_publisher src/arduino_publisher.cpp)
add_executable(pid_publisher src/pid_publisher.cpp)

ament_target_dependencies(arduino_publisher rclcpp std_msgs)
ament_target_dependencies(pid_publisher rclcpp std_msgs)

install(DIRECTORY
    src launch
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    arduino_publisher
    pid_publisher
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

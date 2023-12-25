cmake_minimum_required(VERSION 3.8)
project(serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(${PROJECT_NAME}
    src/main.cpp
    include/serial/connect.hpp
    src/connect.cpp
    include/serial/msg.hpp
    src/msg.cpp
    include/serial/serial_publisher.hpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs)

install(DIRECTORY
    src include
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

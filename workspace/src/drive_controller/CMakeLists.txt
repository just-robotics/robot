cmake_minimum_required(VERSION 3.8)
project(drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_msgs REQUIRED)

add_executable(drive_controller
    src/drive_controller.cpp
)

ament_target_dependencies(drive_controller rclcpp geometry_msgs nav_msgs robot_msgs tf2 tf2_ros)

install(DIRECTORY
    src launch config
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    drive_controller
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.5)
project(cooperative_transport)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(offboard_control_pos src/offboard_control_pos.cpp)
ament_target_dependencies(offboard_control_pos rclcpp px4_msgs)
install(TARGETS offboard_control_pos DESTINATION lib/${PROJECT_NAME})

add_executable(offboard_control_trajectory src/offboard_control_trajectory.cpp)
ament_target_dependencies(offboard_control_trajectory rclcpp px4_msgs std_msgs)
install(TARGETS offboard_control_trajectory DESTINATION lib/${PROJECT_NAME})

add_executable(pid_control src/pid_control.cpp)
ament_target_dependencies(pid_control rclcpp px4_msgs std_msgs geometry_msgs)
install(TARGETS pid_control DESTINATION lib/${PROJECT_NAME})

ament_package()

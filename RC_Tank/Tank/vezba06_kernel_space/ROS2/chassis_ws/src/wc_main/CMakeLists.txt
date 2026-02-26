cmake_minimum_required(VERSION 3.8)
project(wc_main)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(simple_ackermann_controller
  src/simple_ackermann_steering_controller.cpp
  src/SerialCPP.cpp
)

ament_target_dependencies(simple_ackermann_controller
  rclcpp
  ackermann_msgs
)

target_include_directories(simple_ackermann_controller PRIVATE
  include
)

install(TARGETS
  simple_ackermann_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


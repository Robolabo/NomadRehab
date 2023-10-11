cmake_minimum_required(VERSION 3.8)
project(nomad_sensor_emulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(sensor_publisher src/sensor_publisher.cpp)
ament_target_dependencies(sensor_publisher rclcpp sensor_msgs)

install(TARGETS
  sensor_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()

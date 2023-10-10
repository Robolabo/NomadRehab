cmake_minimum_required(VERSION 3.8)
project(nomad_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED 
  src/nomad_hardware_interface.cpp 
  src/nomad_motor_comms.cpp
)

set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)


ament_target_dependencies(${PROJECT_NAME}
  "hardware_interface"
  "controller_manager"
  "serial"
  "rclcpp"
  "pluginlib"
)

add_executable(test_serial 
  src/test_serial.cpp
  src/nomad_motor_comms.cpp
)

ament_target_dependencies(test_serial
  "serial"
  "rclcpp"
)

pluginlib_export_plugin_description_file(hardware_interface nomad_hardware_interface.xml)



install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  controller_manager
  serial
  rclcpp
  pluginlib
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()

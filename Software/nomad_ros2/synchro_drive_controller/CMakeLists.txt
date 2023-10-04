cmake_minimum_required(VERSION 3.16)
project(synchro_drive_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ackermann_msgs
  builtin_interfaces
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  std_srvs
  tf2
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(synchro_drive_controller SHARED
  src/synchro_drive_controller.cpp
  src/odometry.cpp
  src/traction_limiter.cpp
  src/steering_limiter.cpp
)
target_compile_features(synchro_drive_controller PUBLIC cxx_std_17)
target_include_directories(synchro_drive_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/synchro_drive_controller>
)
ament_target_dependencies(synchro_drive_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface synchro_drive_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_synchro_drive_controller
    test/test_synchro_drive_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_synchro_drive_controller.yaml)
  target_link_libraries(test_synchro_drive_controller
    synchro_drive_controller
  )
  ament_target_dependencies(test_synchro_drive_controller
    geometry_msgs
    hardware_interface
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2
    tf2_msgs
  )

  ament_add_gmock(test_load_synchro_drive_controller
    test/test_load_synchro_drive_controller.cpp
  )
  target_link_libraries(test_load_synchro_drive_controller
    synchro_drive_controller
  )
  ament_target_dependencies(test_load_synchro_drive_controller
    controller_manager
    ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/synchro_drive_controller
)
install(
  TARGETS
    synchro_drive_controller
  EXPORT export_synchro_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_synchro_drive_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

cmake_minimum_required(VERSION 3.8)
project(follower_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(diff_drive_controller src/diff_drive_controller.cpp)
ament_target_dependencies(diff_drive_controller
  rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(waypoint_test_publisher src/waypoint_test_publisher.cpp)
ament_target_dependencies(waypoint_test_publisher
  rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(fire_event_bridge src/fire_event_bridge.cpp)
ament_target_dependencies(fire_event_bridge rclcpp std_msgs)

add_executable(fire_mission_manager src/fire_mission_manager.cpp)
ament_target_dependencies(fire_mission_manager
  rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(laser_gpio_driver src/laser_gpio_driver.cpp)
ament_target_dependencies(laser_gpio_driver rclcpp std_msgs)

install(TARGETS
  diff_drive_controller
  waypoint_test_publisher
  fire_event_bridge
  fire_mission_manager
  laser_gpio_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

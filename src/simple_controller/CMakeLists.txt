cmake_minimum_required(VERSION 3.5)
project(simple_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

include_directories(include)

add_library(simple_pure_pursuit_controller SHARED
  src/simple_pure_pursuit_controller.cpp
)

ament_target_dependencies(simple_pure_pursuit_controller
  rclcpp
  rclcpp_lifecycle
  nav2_core
  pluginlib
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  nav2_costmap_2d
)

# REQUIRED FOR PLUGINS
pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS simple_pure_pursuit_controller
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

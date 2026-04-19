cmake_minimum_required(VERSION 3.8)
project(custom_action_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 🔧 Dependencies
find_package(ament_cmake REQUIRED)
find_package(custom_action_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)

# =========================
# COMPONENT (SERVER)
# =========================
add_library(go_to_charger_action_server SHARED
  src/go_to_charger_action_server.cpp
)

target_include_directories(go_to_charger_action_server
  PUBLIC
  include
)

ament_target_dependencies(go_to_charger_action_server
  rclcpp
  rclcpp_action
  rclcpp_components
  custom_action_interfaces
)

rclcpp_components_register_nodes(go_to_charger_action_server
  "custom_action_cpp::GoToChargerActionServer"
)

# =========================
# COMPONENT (CLIENT)
# =========================
add_library(go_to_charger_action_client SHARED
  src/go_to_charger_action_client.cpp
)

target_include_directories(go_to_charger_action_client
  PUBLIC
  include
)

ament_target_dependencies(go_to_charger_action_client
  rclcpp
  rclcpp_action
  rclcpp_components
  custom_action_interfaces
)

rclcpp_components_register_nodes(go_to_charger_action_client
  "custom_action_cpp::GoToChargerActionClient"
)

# =========================
# EXPORT PLUGIN
# =========================
pluginlib_export_plugin_description_file(rclcpp_components plugins.xml)

# =========================
# EXECUTABLES (FOR ros2 run)
# =========================
add_executable(go_to_charger_server
  src/go_to_charger_action_server.cpp
)

ament_target_dependencies(go_to_charger_server
  rclcpp
  rclcpp_action
  custom_action_interfaces
)

add_executable(go_to_charger_client
  src/go_to_charger_action_client.cpp
)

ament_target_dependencies(go_to_charger_client
  rclcpp
  rclcpp_action
  custom_action_interfaces
)

# =========================
# INSTALL
# =========================
install(TARGETS
  go_to_charger_action_server
  go_to_charger_action_client
  go_to_charger_server
  go_to_charger_client
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# 🔧 TESTING
# =========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

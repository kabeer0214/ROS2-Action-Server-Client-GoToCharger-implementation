cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#add_executable(trans src/trans.cpp)
#ament_target_dependencies(trans rclcpp std_msgs geometry_msgs tf2_ros)
#add_executable(talker src/publisher_lambda_function.cpp)
#ament_target_dependencies(talker rclcpp std_msgs)
add_executable(broad src/broad.cpp)
ament_target_dependencies(broad rclcpp std_msgs geometry_msgs tf2_ros tf2)
#add_executable(act src/act.cpp)
#ament_target_dependencies(act rclcpp std_msgs)
install(TARGETS
  #talker
  #trans
  broad
  DESTINATION lib/${PROJECT_NAME})

ament_package()

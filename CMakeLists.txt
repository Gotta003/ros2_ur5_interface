cmake_minimum_required(VERSION 3.8)
project(ros2_ur5_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(publish_trajectory_node src/publish_trajectory_node.cpp)

ament_target_dependencies(publish_trajectory_node
  rclcpp
  trajectory_msgs
  sensor_msgs
)

install(TARGETS
  publish_trajectory_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory
install(
  DIRECTORY 
  launch
  models
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

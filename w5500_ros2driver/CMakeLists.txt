cmake_minimum_required(VERSION 3.8)
project(w5500_ros2driver)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(w5500_msg REQUIRED)

# Path to your .cpp file
add_executable(force_publisher
  w5500_ros2driver/w5500.cpp
)

# Link dependencies
ament_target_dependencies(force_publisher
  rclcpp
  w5500_msg
)

# Install binary so ROS2 can find it
install(TARGETS
  force_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

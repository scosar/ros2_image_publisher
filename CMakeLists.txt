cmake_minimum_required(VERSION 3.5)
project(ros2_image_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(image_publisher src/image_publisher.cpp)

# Link libraries
ament_target_dependencies(image_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

# Install
install(TARGETS
  image_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

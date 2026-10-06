cmake_minimum_required(VERSION 3.8)
project(robocup_vision_core)

# Include header files
include_directories(include/${PROJECT_NAME})

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robocup_vision_interfaces REQUIRED)

# Build executable
add_executable(camera_subscriber
  src/main.cpp
  src/camera_subscriber.cpp
  src/crop_space.cpp
  src/voxelization.cpp
  src/ransac_plane.cpp
  src/dbscan_clustering.cpp
  src/obb_estimator.cpp
  src/plana_obb_estimator.cpp
  src/obb_visualizer.cpp
  src/color_utils.cpp
  src/classifier.cpp
  src/classification_visualizer.cpp
  src/block_registry.cpp
)

ament_target_dependencies(camera_subscriber
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
  robocup_vision_interfaces
)

# Install
install(TARGETS
  camera_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


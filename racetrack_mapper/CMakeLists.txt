cmake_minimum_required(VERSION 3.5)
project(racetrack_mapper)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Add executable
add_executable(racetrack_mapper src/racetrack_mapper.cpp)

# Link libraries
ament_target_dependencies(racetrack_mapper
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  visualization_msgs)

target_link_libraries(racetrack_mapper
  ${PCL_LIBRARIES}
)

# Install the executable
install(TARGETS racetrack_mapper
  DESTINATION lib/${PROJECT_NAME})

# Ament package
ament_package()

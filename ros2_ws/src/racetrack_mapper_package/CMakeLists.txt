cmake_minimum_required(VERSION 3.8)
project(racetrack_mapper_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# PCL include + lib
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# process_data_node (PCL kell)
add_executable(process_data_node src/data_process.cpp)
ament_target_dependencies(process_data_node
  rclcpp std_msgs sensor_msgs pcl_conversions pcl_ros
)
target_link_libraries(process_data_node ${PCL_LIBRARIES})

# tf_broadcaster_node (PCL NEM kell)
add_executable(tf_broadcaster_node src/broadcaster.cpp)
ament_target_dependencies(tf_broadcaster_node
  rclcpp std_msgs sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs ament_index_cpp
)

install(TARGETS
  process_data_node
  tf_broadcaster_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
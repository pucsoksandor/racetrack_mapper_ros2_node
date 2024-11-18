# racetrack_mapper_ros2_node
This reepository contains my assignment for HUMDA Lab Nonprofit Kft.

# Racetrack Mapper Node

This ROS 2 node, `RacetrackMapper`, processes lidar data to create a map of a racetrack and visualizes the vehicle's position. The node subscribes to the vehicle's pose and lidar point cloud topics, applies transformations, filters the point cloud using a VoxelGrid filter, and publishes the transformed and filtered data along with a visualization marker for the vehicle's position.

## Features
- **Vehicle Pose Subscription**: Subscribes to `/a2rl/state_estimation/ego_loc_fused` to receive the vehicle's pose.
- **Lidar Data Subscription**: Subscribes to `/sensor/lidar_front/points` to receive the lidar point cloud data.
- **Static Transform Publication**: Publishes static transforms between the `map`, `base_link`, and `lidar_front` frames.
- **VoxelGrid Filtering**: Applies a VoxelGrid filter to downsample the lidar data for efficient processing.
- **Point Cloud Transformation**: Transforms the lidar point cloud data into the `map` frame using `tf2`.
- **Map Accumulation**: Accumulates filtered lidar data into a racetrack map and periodically publishes it.
- **Vehicle Position Visualization**: Publishes a visualization marker for the vehicle’s position in the map.

## How It Works

1. **Pose Callback**:
   - Receives the vehicle's pose data and logs the position.
   - Publishes a marker showing the vehicle's position on the map.

2. **Lidar Callback**:
   - Waits for the vehicle pose data to be available.
   - Transforms the incoming lidar point cloud into the `map` frame.
   - Applies the VoxelGrid filter to downsample the point cloud.
   - Accumulates the filtered point cloud into the racetrack map.
   - Publishes both the transformed lidar point cloud and the updated racetrack map.

3. **Static Transform Broadcasting**:
   - Publishes static transforms between the `map` frame and `base_link`, as well as from `base_link` to `lidar_front`.

4. **Signal Handling**:
   - Gracefully shuts down the node when a `SIGINT` (Ctrl+C) signal is received.

## Dependencies
- ROS 2 (Humble or later)
- `sensor_msgs`
- `geometry_msgs`
- `visualization_msgs`
- `tf2_ros`
- `pcl_ros`
- `pcl_conversions`

# 1. Node Implementation:

- For the first time I have installed a Linux WSL, where I can do the job properly.
- Then I installed the Humble version of ROS2 and made all the necessary adjustments.
- I set it to run the following lines at the start of each terminal: `source /opt/ros/humble/setup.bash` `source ~/ros2_ws/install/setup.bash`
- I made a ros2 workspace and built it.
- And then I created a C++ package in a src folder and built it, this is where I will implement the node. The name of the node is racetrack_mapper.

# 2. Visualization:

- For visualization i choose Foxglove, because Rviz2 always crashed.
- To visualize the data I had to start a bridge with the following line `ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765`

- The following figure shows how the transformed data is displayed in the visualisation software.
  ![map](https://github.com/user-attachments/assets/ac2c550f-71cf-4f53-8625-2819062ba4b4)

- And in this pictures you can see the drawn map, unfortunately I didn't get the expected slope, but under the visualization you can see that the mapper rotates the points correctly, but all the points are stacked on top of each other.
![joooooooooooo](https://github.com/user-attachments/assets/fa0f9076-bd09-4a97-b3fe-34636be818d4)
![ez lesz jo](https://github.com/user-attachments/assets/239e78b9-3f8b-4e72-986b-72e4bc8b3614)
![ez isjo lesz](https://github.com/user-attachments/assets/4915d171-31e5-4f29-bc84-b048bf877431)

- Possible problems: The foxglove bridge send the following warning '[foxglove_bridge-1] [WARN] [1731934131.377791179] [foxglove_bridge]: [WS] 127.0.0.1:34514: Send buffer limit reached'.
- Problems with transformation to map coordinate system

# 3. Algorithm Analysis:

- I used the VoxelGrid filter to optimize my code by downsampling the point cloud. This filter reduces the number of points in the cloud, making the data easier to process and improving performance. It divides the space into voxels (3D grid cells) and represents each voxel with a single point, thus reducing the point density while preserving the overall structure of the cloud. I can control the level of downsampling by adjusting the voxel size to 1.

# Racetrack Lidar Map Builder

This project builds a **continuous, global 3D point‑cloud map** from a moving vehicle’s LiDAR sensor using ROS2 Humble.  
The node subscribes to `/sensor/lidar_front/points`, performs **deskewing**, transforms all points into the `map` frame, accumulates them into a global map, applies **voxel downsampling**, and publishes the result to `/world/transformed_point_cloud`.

The map can be visualized live in **Foxglove Studio**.

<img width="1524" height="1012" alt="Képernyőfotó 2026-03-20 - 0 47 58" src="https://github.com/user-attachments/assets/2365c552-1168-4b1b-962b-b3bf889af27a" />

## Features

### ✔ LiDAR → map transformation
Uses the TF chain:
map ← baselink ← lidar_front

Every incoming LiDAR frame is transformed into the global `map` coordinate system.

### ✔ Deskewing (motion compensation)
LiDAR scans are not captured in a single instant.  
The node assigns a timestamp offset to each point and transforms it using the correct TF for that moment.  
This prevents the map from becoming “smeared” or “broken”.

### ✔ Global map accumulation
All deskewed frames are appended to a global PCL point cloud.

### ✔ VoxelGrid downsampling
To keep the map size manageable, a PCL `VoxelGrid` filter is applied.  
Typical leaf sizes: `0.1–0.3 m`.

### ✔ ROS2 publishing
The final map is published as:
/world/transformed_point_cloud

Frame: `map`  
Type: `sensor_msgs/msg/PointCloud2`

---

## Project Structure

ros2_ws/
├── src/
│    └── racetrack_mapper/
│          ├── CMakeLists.txt
│          ├── package.xml
│          └── src/
│               └── lidar_map_builder.cpp
└── Dockerfile


---

## Parameters

| Parameter       | Type   | Description |
|-----------------|--------|-------------|
| `scan_duration` | double | Estimated duration of one LiDAR scan (used for deskewing). Default: `0.1` seconds. |

---

## Build & Run with Docker

### 1) Build the Docker image

From the workspace root:

```bash
docker build --no-cache -t racetrack_mapper .

docker run -it -p 8765:8765 racetrack_mapper


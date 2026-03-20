FROM ros:humble

# Install required ROS and system packages
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-ament-index-python \
    ros-humble-pcl-ros \
    ros-humble-foxglove-bridge \
    ros-humble-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Set workspace
WORKDIR /ros2_ws

# Copy source code
COPY ./src/racetrack_mapper_package ./src/racetrack_mapper_package

# Install dependencies
RUN rosdep install --from-paths src --ignore-src -r -y

# Build workspace
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Use bash for entrypoint
SHELL ["/bin/bash", "-c"]

# Source environment and launch
ENTRYPOINT source /opt/ros/humble/setup.bash && \
           source /ros2_ws/install/setup.bash && \
           ros2 launch racetrack_mapper_package launcher.py
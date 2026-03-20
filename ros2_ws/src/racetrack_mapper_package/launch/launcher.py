from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

# Get package directory
pkg_dir = get_package_share_directory('racetrack_mapper_package')

# Paths
measurement_dir = '/ros2_ws/src/racetrack_mapper_package/meas'
bag_file = os.path.join(measurement_dir, 'your.mcap')  # add your bag file
static_map_file = os.path.join(measurement_dir, 'your.yaml') #  add your map file

def generate_launch_description():
    ld = LaunchDescription()

    # Data processing node
    process_node = Node(
        package='racetrack_mapper_package',
        executable='process_data_node',
        name='process_data_node',
        parameters=[{
            'use_sim_time': True,
            'static_map_path': static_map_file
        }],
        output='screen'
    )

    # TF broadcaster node
    tf_node = Node(
        package='racetrack_mapper_package',
        executable='tf_broadcaster_node',
        name='tf_broadcaster_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Static transform: baselink -> lidar_front
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_node',
        arguments=[
            '--x', '0.63', '--y', '0.0', '--z', '0.0',
            '--roll', '3.141592653589793', '--pitch', '-1.5707963267948965', '--yaw', '0.0',
            '--frame-id', 'baselink',
            '--child-frame-id', 'lidar_front'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Play ROS2 bag with simulated time
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-s', 'mcap', '--clock', '1', bag_file],
        output='screen'
    )

    # Foxglove bridge for visualization
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        arguments=[
            '--websocket-port', '8765',
            '--auto-start'
        ]
    )

    # Debug: print nodes and topics periodically
    debug_nodes = ExecuteProcess(
        cmd=['bash', '-c', 'while true; do echo "--- ROS2 NODES ---"; ros2 node list; echo "--- ROS2 TOPICS ---"; ros2 topic list; sleep 5; done'],
        output='screen'
    )

    # Add all actions to launch
    ld.add_action(process_node)
    ld.add_action(tf_node)
    ld.add_action(static_tf_node)
    ld.add_action(bag_play)
    ld.add_action(foxglove_node)
    ld.add_action(debug_nodes)

    return ld
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>
#include <signal.h>

class RacetrackMapper : public rclcpp::Node
{
public:
    // Constructor to initialize the node and set up the subscriptions, publishers, and transform broadcasters
    RacetrackMapper() : Node("racetrack_mapper"),
                        tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
                        map_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        // Subscription to receive the pose data of the vehicle
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/a2rl/state_estimation/ego_loc_fused", 10, std::bind(&RacetrackMapper::pose_callback, this, std::placeholders::_1));

        // Subscription to receive the lidar point cloud data
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/lidar_front/points", 10, std::bind(&RacetrackMapper::lidar_callback, this, std::placeholders::_1));

        // Publisher for transformed lidar points
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/transformed_lidar_points", 10);

        // Publisher for the accumulated map
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/racetrack_map", 10);

        // Publisher for the vehicle position marker
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/vehicle_position_marker", 10);

        // Initialize the static transform broadcaster
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Publish static transforms for the map and lidar frames
        publish_static_map_to_base_link_transform();
        publish_static_lidar_to_base_link_transform();

        RCLCPP_INFO(this->get_logger(), "Racetrack Mapper Node started.");
    }

private:
    // Callback for receiving and handling the vehicle's pose data
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        current_pose_ = msg; // Store the current pose data
        RCLCPP_INFO(this->get_logger(), "Received pose: [x: %f, y: %f, z: %f]", 
                    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        publish_vehicle_position(); // Publish the vehicle's position as a marker
    }

    // Callback for receiving and handling the lidar point cloud data
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!current_pose_)
        {
            RCLCPP_WARN(this->get_logger(), "Pose not available yet");
            return;
        }

        // Transform the lidar points from the lidar frame to the map frame
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_.lookupTransform("map", "lidar_front", msg->header.stamp);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
            return;
        }

        // Apply the transformation to the lidar point cloud
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        tf2::doTransform(*msg, transformed_cloud, transform_stamped);

        // Convert ROS point cloud message to PCL point cloud for processing
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(transformed_cloud, *pcl_cloud);

        // Apply a voxel grid filter to downsample the point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(pcl_cloud);
        voxel_filter.setLeafSize(1.0f, 1.0f, 1.0f); //set 1
        voxel_filter.filter(*filtered_cloud);

        // Lock map access to avoid concurrency issues and update the accumulated map
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            *map_cloud_ += *filtered_cloud;
        }

        // Publish the transformed lidar points
        cloud_pub_->publish(transformed_cloud);

        // Publish the accumulated map as a point cloud
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*map_cloud_, output);
        output.header.frame_id = "map";
        map_pub_->publish(output);
        RCLCPP_INFO(this->get_logger(), "Published accumulated racetrack map");
    }

    // Publish a static transform from the map frame to the base_link frame
    void publish_static_map_to_base_link_transform()
    {
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->get_clock()->now();
        static_transform.header.frame_id = "map";
        static_transform.child_frame_id = "base_link";
        static_transform.transform.translation.x = 0.0;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;
        static_transform.transform.rotation.x = 0.0;
        static_transform.transform.rotation.y = 0.0;
        static_transform.transform.rotation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;
        static_broadcaster_->sendTransform(static_transform);

        RCLCPP_INFO(this->get_logger(), "Published static transform from map to base_link");
    }

    // Publish a static transform from the base_link frame to the lidar_front frame
    void publish_static_lidar_to_base_link_transform()
    {
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->get_clock()->now();
        static_transform.header.frame_id = "base_link";
        static_transform.child_frame_id = "lidar_front";
        static_transform.transform.translation.x = 0.63;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;
        static_transform.transform.rotation.x = 0.0;
        static_transform.transform.rotation.y = 0.0;
        static_transform.transform.rotation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;
        static_broadcaster_->sendTransform(static_transform);

        RCLCPP_INFO(this->get_logger(), "Published static transform from base_link to lidar_front");
    }

    // Publish a marker representing the vehicle's position
    void publish_vehicle_position()
    {
        if (!current_pose_)
            return;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "vehicle_position";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = current_pose_->pose.pose;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_pub_->publish(marker);
    }

    // Subscribers, publishers, and transform objects
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> current_pose_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    std::mutex map_mutex_;
};

// Shared pointer to the node instance
std::shared_ptr<RacetrackMapper> node;

// Signal handler for shutting down the node gracefully
void signal_handler(int signum)
{
    RCLCPP_INFO(node->get_logger(), "Signal %d received, shutting down...", signum);
    rclcpp::shutdown();
}

// Main function to initialize and spin the ROS 2 node
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // Initialize ROS 2
    node = std::make_shared<RacetrackMapper>(); // Create the node instance

    // Set up the signal handler for graceful shutdown
    signal(SIGINT, signal_handler);

    rclcpp::spin(node);  // Spin the node to process incoming messages

    return 0;
}

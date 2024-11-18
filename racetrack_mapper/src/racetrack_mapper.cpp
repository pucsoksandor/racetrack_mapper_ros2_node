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
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>
#include <signal.h>

class RacetrackMapper : public rclcpp::Node
{
public:
    RacetrackMapper() : Node("racetrack_mapper"),
                        tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
                        map_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        // Subscriber to vehicle pose (position and orientation)
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/a2rl/state_estimation/ego_loc_fused", 10, std::bind(&RacetrackMapper::pose_callback, this, std::placeholders::_1));

        // Subscriber to LiDAR point cloud data
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/lidar_front/points", 10, std::bind(&RacetrackMapper::lidar_callback, this, std::placeholders::_1));

        // Publisher for transformed LiDAR point clouds
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/transformed_lidar_points", 10);

        // Publisher for the accumulated racetrack map
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/racetrack_map", 10);

        // Publisher for visualizing the vehicle's current position
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/vehicle_position_marker", 10);

        // Initialize a static transform broadcaster for fixed frame transformations
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Define and broadcast the static transform for the LiDAR sensor relative to the vehicle
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->get_clock()->now();
        static_transform.header.frame_id = "base_link"; // The vehicle's base frame
        static_transform.child_frame_id = "lidar_front"; // The LiDAR sensor's frame
        static_transform.transform.translation.x = 0.63; // LiDAR sensor's offset in x-axis
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;
        static_transform.transform.rotation.x = 0.0;
        static_transform.transform.rotation.y = 0.0;
        static_transform.transform.rotation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;
        static_broadcaster_->sendTransform(static_transform);

        // Broadcast a static transform between the map and base_link frames
        publish_static_map_to_base_link_transform();

        RCLCPP_INFO(this->get_logger(), "Racetrack Mapper Node started.");
    }

private:
    // Callback for receiving vehicle pose updates
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        current_pose_ = msg; // Update the current pose
        RCLCPP_INFO(this->get_logger(), "Received pose: [x: %f, y: %f, z: %f]", 
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        // Publish the vehicle position as an RViz marker
        publish_vehicle_position();
    }

    // Callback for receiving LiDAR point cloud data
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!current_pose_)
        {
            RCLCPP_WARN(this->get_logger(), "Pose not available yet");
            return;
        }

        // Transform the LiDAR point cloud into the map frame
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

        sensor_msgs::msg::PointCloud2 transformed_cloud;
        tf2::doTransform(*msg, transformed_cloud, transform_stamped);

        // Convert the ROS point cloud message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(transformed_cloud, pcl_cloud);

        // Add new points to the accumulated map
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            for (std::size_t i = 0; i < pcl_cloud.points.size(); i += 10)
            {
                map_cloud_->points.push_back(pcl_cloud.points[i]);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Transformed LiDAR point cloud with width: %d, height: %d", transformed_cloud.width, transformed_cloud.height);

        // Publish the transformed point cloud
        cloud_pub_->publish(transformed_cloud);

        // Convert the accumulated map back to a ROS message and publish it
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*map_cloud_, output);
        output.header.frame_id = "map";
        map_pub_->publish(output);

        RCLCPP_INFO(this->get_logger(), "Published accumulated racetrack map");
    }

    // Publish a static transform between the map and base_link frames
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

    // Publish the vehicle's position as an RViz marker
    void publish_vehicle_position()
    {
        if (!current_pose_) return;

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

    // Subscribers and publishers
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // Current pose of the vehicle
    std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> current_pose_;

    // Transform-related objects
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    // Accumulated map as a PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;

    // Mutex for thread-safe access to the map cloud
    std::mutex map_mutex_;
};

// Global pointer to the node instance
std::shared_ptr<RacetrackMapper> node;

// Signal handler for graceful shutdown
void signal_handler(int signum)
{
    RCLCPP_INFO(node->get_logger(), "Signal %d received, shutting down...", signum);
    rclcpp::shutdown();
}

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<RacetrackMapper>();

    signal(SIGINT, signal_handler);

    rclcpp::spin(node);

    return 0;
}

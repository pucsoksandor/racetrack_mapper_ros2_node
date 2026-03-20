#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Geometry>

class LidarMapBuilderNode : public rclcpp::Node
{
public:
  LidarMapBuilderNode()
  : Node("lidar_map_builder_node")
  {
    // Estimated scan duration (seconds)
    scan_duration_ = this->declare_parameter<double>("scan_duration", 0.1);

    // Subscribe to LiDAR point cloud
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/sensor/lidar_front/points", 10,
      std::bind(&LidarMapBuilderNode::handlePointCloud, this, std::placeholders::_1));

    // Publisher for global map
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/world/transformed_point_cloud", 10);

    // TF listener setup
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Periodic map publishing
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&LidarMapBuilderNode::publishMap, this));

    // Initialize point clouds
    accumulated_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    RCLCPP_INFO(this->get_logger(), "LidarMapBuilderNode initialized");
  }

private:
  void handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Skip empty messages
    if (msg->width == 0) {
      return;
    }

    // Check TF availability
    rclcpp::Time stamp = msg->header.stamp;
    if (!tf_buffer_->canTransform("map", "lidar_front", stamp)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No TF map<-lidar_front at cloud time");
      return;
    }

    // Convert ROS -> PCL
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);

    const std::size_t N = cloud.points.size();
    if (N == 0) {
      return;
    }

    // Linear time interpolation across scan
    double dt = scan_duration_ / static_cast<double>(N);

    pcl::PointCloud<pcl::PointXYZI> cloud_deskewed;
    cloud_deskewed.points.reserve(N);

    for (std::size_t i = 0; i < N; ++i) {
      double offset = dt * static_cast<double>(i);
      rclcpp::Time pt_time = stamp + rclcpp::Duration::from_seconds(offset);

      geometry_msgs::msg::TransformStamped tf;
      try {
        // Lookup transform at point time
        tf = tf_buffer_->lookupTransform(
          "map", "lidar_front", pt_time,
          tf2::durationFromSec(0.05));
      } catch (const tf2::TransformException &ex) {
        // Fallback to scan timestamp
        try {
          tf = tf_buffer_->lookupTransform(
            "map", "lidar_front", stamp);
        } catch (...) {
          continue;
        }
      }

      const auto &p = cloud.points[i];

      // Point in LiDAR frame
      Eigen::Vector3d pt_lidar(p.x, p.y, p.z);

      // Rotation and translation
      Eigen::Quaterniond q_map(
        tf.transform.rotation.w,
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z);

      Eigen::Vector3d t_map(
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z);

      // Transform to map frame
      Eigen::Vector3d pt_map = q_map * pt_lidar + t_map;

      pcl::PointXYZI out;
      out.x = static_cast<float>(pt_map.x());
      out.y = static_cast<float>(pt_map.y());
      out.z = static_cast<float>(pt_map.z());
      out.intensity = p.intensity;

      cloud_deskewed.points.push_back(out);
    }

    cloud_deskewed.width = cloud_deskewed.points.size();
    cloud_deskewed.height = 1;
    cloud_deskewed.is_dense = false;

    // Accumulate into global map
    *accumulated_cloud_ += cloud_deskewed;

    // Downsample using voxel grid
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(accumulated_cloud_);
    voxel.setLeafSize(0.8, 0.8, 0.8);
    voxel.filter(*voxel_cloud_);
  }

  void publishMap()
  {
    // Skip if empty
    if (!voxel_cloud_ || voxel_cloud_->empty()) {
      return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*voxel_cloud_, msg);
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";

    publisher_->publish(msg);
  }

  // Parameters
  double scan_duration_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Point clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarMapBuilderNode>());
  rclcpp::shutdown();
  return 0;
}
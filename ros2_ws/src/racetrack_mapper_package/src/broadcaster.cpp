#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher(): Node("tf2_frame_broadcaster")
  {
    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    std::string topic_name = "/a2rl/state_estimation/ego_loc_fused";
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>( topic_name, 10, 
        std::bind(&FramePublisher::handle_pose, this, std::placeholders::_1));
  }

private:
  void handle_pose(const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> msg)
  {
    geometry_msgs::msg::TransformStamped transform;

    // Read message content and assign it to corresponding tf variables
    transform.header = msg->header;
    transform.header.frame_id = "map";
    transform.child_frame_id = "baselink";
    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = msg->pose.pose.position.z;
    transform.transform.rotation = msg->pose.pose.orientation;

    // Send the transformation
    tf_broadcaster_->sendTransform(transform);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string turtlename_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}
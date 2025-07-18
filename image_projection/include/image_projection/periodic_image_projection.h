#ifndef IMAGE_PROJECTION_PERIODIC_IMAGE_PROJECTION_H
#define IMAGE_PROJECTION_PERIODIC_IMAGE_PROJECTION_H

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
// #include <dynamic_reconfigure/server.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>

#include <image_projection_msgs/srv/project_pixel_to3_d_ray.hpp>

// #include <image_projection/ProjectionConfig.h>
#include <image_projection/image_projection.h>

namespace image_projection {

class PeriodicImageProjection
{
public:
  PeriodicImageProjection(const rclcpp::Node::SharedPtr node);
  bool init();
  void initProjectionMat();
  void projectAndPublishLatestImages();

private:
  void connectCb();
  void poseParamCallback(std::vector<double> pose_vec);
  void poseCallback(const std::shared_ptr<geometry_msgs::msg::Pose const> pose);
  void updateSensorPose(const Eigen::Isometry3d& sensor_pose);
  void updateSensorPose(double x, double y, double z, double roll, double pitch, double yaw);
  bool projectPixelToRayCb(const image_projection_msgs::srv::ProjectPixelTo3DRay::Request::SharedPtr req,
                           image_projection_msgs::srv::ProjectPixelTo3DRay::Response::SharedPtr resp);
  void publishTfTimerCallback();
  void publishCameraFrameToTf();

  // Node
  rclcpp::Node::SharedPtr node_;
  bool enabled_;

  std::recursive_mutex reconfigure_mutex_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Service<image_projection_msgs::srv::ProjectPixelTo3DRay>::SharedPtr pixel_to_ray_srv_;

  // TF
  bool publish_tf_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  geometry_msgs::msg::TransformStamped optical_transform_msg_;
  rclcpp::Time last_tf_stamp_;

  // Image projection
  ImageProjection image_projection_lib_;
  ProjectionPtr projection_;
  PixelMapping pixel_mapping_;
  cv::UMat projected_image_;
  rclcpp::Time last_image_stamp_{};

  // Image publisher
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  std::mutex connect_mutex_;

  // Parameter Subscription for pose
  hector::ParameterSubscription pose_param_sub_;
  std::vector<double> pose_vec_;

  // Parameters
  std::string base_frame_;
  std::string virtual_sensor_frame_;
  std::string virtual_sensor_optical_frame_;
  Eigen::Isometry3d virtual_sensor_pose_;
  Eigen::Isometry3d virtual_sensor_optical_pose_;
  Eigen::Isometry3d optical_frame_transform_;
  std::string encoding_;
  bool always_recompute_mapping_;

  // TODO remove once image transport adds callback to advertise
  rclcpp::TimerBase::SharedPtr connection_check_timer_;
};

}  // namespace image_projection

#endif

#ifndef IMAGE_PROJECTION_PERIODIC_IMAGE_PROJECTION_H
#define IMAGE_PROJECTION_PERIODIC_IMAGE_PROJECTION_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_broadcaster.h>

#include <image_projection_msgs/ProjectPixelTo3DRay.h>

#include <image_projection/ProjectionConfig.h>
#include <image_projection/image_projection.h>

namespace image_projection {

class PeriodicImageProjection {
public:
  PeriodicImageProjection(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  bool init();
  void initProjectionMat();
  void projectAndPublishLatestImages();
private:
  void connectCb();
  void dynamicReconfigureCallback(ProjectionConfig &config, uint32_t /*level*/);
  void poseCallback(const geometry_msgs::PoseConstPtr& pose);
  void updateSensorPose(const Eigen::Isometry3d& sensor_pose);
  void updateSensorPose(double x, double y, double z, double roll, double pitch, double yaw);
  void updateReconfigureConfig(const std::vector<double>& pose_vec);
  bool projectPixelToRayCb(image_projection_msgs::ProjectPixelTo3DRay::Request& req, image_projection_msgs::ProjectPixelTo3DRay::Response &resp);
  void publishTfTimerCallback(const ros::TimerEvent& /*event*/);
  void publishCameraFrameToTf();

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  bool enabled_;

  // Dynamic reconfigure
  std::shared_ptr<dynamic_reconfigure::Server<image_projection::ProjectionConfig>> reconfigure_server_;
  boost::recursive_mutex reconfigure_mutex_;
  ros::Subscriber pose_sub_;
  ros::ServiceServer pixel_to_ray_srv_;

  // TF
  bool publish_tf_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Timer tf_timer_;
  geometry_msgs::TransformStamped optical_transform_msg_;
  ros::Time last_tf_stamp_;

  // Image projection
  ImageProjection image_projection_lib_;
  ProjectionPtr projection_;
  PixelMapping pixel_mapping_;
  cv::UMat projected_image_;

  // Image publisher
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  boost::mutex connect_mutex_;

  // Parameters
  std::string base_frame_;
  std::string virtual_sensor_frame_;
  std::string virtual_sensor_optical_frame_;
  Eigen::Isometry3d virtual_sensor_pose_;
  Eigen::Isometry3d virtual_sensor_optical_pose_;
  Eigen::Isometry3d optical_frame_transform_;
  std::string encoding_;
  bool always_recompute_mapping_;
};

}

#endif

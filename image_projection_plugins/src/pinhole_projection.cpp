#include <image_projection_plugins/pinhole_projection.h>
#include <pluginlib/class_list_macros.h>

namespace image_projection_plugins {

bool PinholeProjection::initialize(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
{
  nh_ = nh;
  pnh_ = pnh;
  pnh_.param("virtual_sensor_optical_frame", virtual_sensor_optical_frame_, std::string(""));
  if (!virtual_sensor_optical_frame_.empty()) {
    camera_info_pub_ = pnh_.advertise<sensor_msgs::CameraInfo>("camera_info", 10, true);
  }
  return true;
}

Eigen::Vector2d PinholeProjection::projectionSurfacePointToTargetImagePixel(const Eigen::Vector3d& point) const
{
  // TODO implement
  ROS_WARN_STREAM("Not implemented");
  return Eigen::Vector2d::Zero();
}

Eigen::Vector3d PinholeProjection::targetImagePixelToProjectionSurfacePoint(const Eigen::Vector2d& target_image_pixel) const
{
  Eigen::Vector3d point;
  // camera frame
//  point.x() = focal_length_;
//  point.y() = (image_width_2_ - target_image_pixel.x()) * m_per_pixel_;
//  point.z() =
  // optical frame
  point.x() = (target_image_pixel.x() - image_width_2_) * m_per_pixel_;
  point.y() = (target_image_pixel.y() - image_height_2_) * m_per_pixel_;
  point.z() = focal_length_;
  return point;
}

bool PinholeProjection::loadProjectionParametersFromNamespace(const ros::NodeHandle& nh)
{
  registerParameterFromNamespace(nh, "focal_length", 1, "Focal length", 0, 10);
  registerParameterFromNamespace(nh, "horizontal_fov", 90, "Horizontal field of view in degree", 0, 179);
  parametersChanged();
  return true;
}

void PinholeProjection::parametersChanged()
{
  focal_length_ = getParameter("focal_length");
  double horizontal_fov_rad = getParameter("horizontal_fov") * M_PI / 180;
  double sensor_size_x = 2 * focal_length_ * std::tan(horizontal_fov_rad / 2.0);
  m_per_pixel_ = sensor_size_x / imageWidth();
  image_width_2_ = static_cast<double>(imageWidth()) / 2.0;
  image_height_2_ = static_cast<double>(imageHeight()) / 2.0;
  publishCameraInfo();
}

void PinholeProjection::publishCameraInfo()
{
  if (virtual_sensor_optical_frame_.empty()) {
    return;
  }
  sensor_msgs::CameraInfo info = parametersToCameraInfo();
  camera_info_pub_.publish(info);
}

sensor_msgs::CameraInfo PinholeProjection::parametersToCameraInfo() const
{
  sensor_msgs::CameraInfo info_msg;
  info_msg.header.stamp = ros::Time::now();
  info_msg.header.frame_id = virtual_sensor_optical_frame_;

  info_msg.width = static_cast<unsigned int>(imageWidth());
  info_msg.height = static_cast<unsigned int>(imageHeight());

  double principle_point_x = image_width_2_ + 0.5;
  double principle_point_y = image_height_2_ + 0.5;
  double focal_length_pixel = focal_length_ / m_per_pixel_;
  info_msg.K[0] = info_msg.K[4] = focal_length_pixel;
  info_msg.K[2] = principle_point_x;
  info_msg.K[5] = principle_point_y;
  info_msg.K[8] = 1;

  info_msg.P[0] = info_msg.P[5] = focal_length_pixel;
  info_msg.P[2] = principle_point_x;
  info_msg.P[6] = principle_point_y;
  info_msg.P[10] = 1;

  return info_msg;
}

}

PLUGINLIB_EXPORT_CLASS(image_projection_plugins::PinholeProjection, image_projection_plugin_interface::ProjectionBase)

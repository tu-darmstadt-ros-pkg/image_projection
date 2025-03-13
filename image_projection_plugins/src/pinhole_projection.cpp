#include <image_projection_plugins/pinhole_projection.h>

namespace image_projection_plugins {

bool PinholeProjection::initialize(const rclcpp::Node::SharedPtr& node, const std::string& name)
{
  ProjectionBase::initialize(node, name);

  addReconfigurableParameter("virtual_sensor_optical_frame", virtual_sensor_optical_frame_,
                             "Name of the optical frame to be published for the virtual camera.");
  if (!virtual_sensor_optical_frame_.empty()) {
    camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);  // TODO transient local
  }
  return true;
}

Eigen::Vector2d PinholeProjection::projectionSurfacePointToTargetImagePixel(const Eigen::Vector3d& point) const
{
  // TODO implement
  RCLCPP_ERROR(node_->get_logger(), "Not implemented");
  return Eigen::Vector2d::Zero();
}

Eigen::Vector3d
PinholeProjection::targetImagePixelToProjectionSurfacePoint(const Eigen::Vector2d& target_image_pixel) const
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

bool PinholeProjection::loadProjectionParameters()
{
  addReconfigurableParameter(
      "focal_length", focal_length_, "Focal length of the virtual camera (in m)",
      hector::ReconfigurableParameterOptions<double>().onValidate([](const auto& value) { return value > 0; }));
  addReconfigurableParameter("horizontal_fov", horizontal_fov, "Horizontal field of view (in degree)",
                             hector::ReconfigurableParameterOptions<double>().onValidate(
                                 [](const auto& value) { return value > 0 && value < 180; }));

  return true;
}

void PinholeProjection::onParametersChanged()
{
  const double horizontal_fov_rad = horizontal_fov * M_PI / 180;
  const double sensor_size_x = 2 * focal_length_ * std::tan(horizontal_fov_rad / 2.0);
  m_per_pixel_ = sensor_size_x / imageWidth();
  image_width_2_ = static_cast<double>(imageWidth()) / 2.0;
  image_height_2_ = static_cast<double>(imageHeight()) / 2.0;
  publishCameraInfo();
}

void PinholeProjection::publishCameraInfo() const
{
  if (virtual_sensor_optical_frame_.empty()) {
    return;
  }
  const sensor_msgs::msg::CameraInfo info = parametersToCameraInfo();
  camera_info_pub_->publish(info);
}

sensor_msgs::msg::CameraInfo PinholeProjection::parametersToCameraInfo() const
{
  sensor_msgs::msg::CameraInfo info_msg;
  info_msg.header.stamp = node_->get_clock()->now();
  info_msg.header.frame_id = virtual_sensor_optical_frame_;

  info_msg.width = static_cast<unsigned int>(imageWidth());
  info_msg.height = static_cast<unsigned int>(imageHeight());

  const double principle_point_x = image_width_2_ + 0.5;
  const double principle_point_y = image_height_2_ + 0.5;
  const double focal_length_pixel = focal_length_ / m_per_pixel_;
  info_msg.k[0] = info_msg.k[4] = focal_length_pixel;
  info_msg.k[2] = principle_point_x;
  info_msg.k[5] = principle_point_y;
  info_msg.k[8] = 1;

  info_msg.p[0] = info_msg.p[5] = focal_length_pixel;
  info_msg.p[2] = principle_point_x;
  info_msg.p[6] = principle_point_y;
  info_msg.p[10] = 1;

  return info_msg;
}

}  // namespace image_projection_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(image_projection_plugins::PinholeProjection, image_projection_plugin_interface::ProjectionBase)

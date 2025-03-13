#include <image_projection_plugins/cylindrical/mercator_projection.h>

namespace image_projection_plugins {

Eigen::Vector2d MercatorProjection::projectionSurfacePointToTargetImagePixel(const Eigen::Vector3d& point) const
{
  // TODO implement
  RCLCPP_ERROR(node_->get_logger(), "Not implemented");
  return Eigen::Vector2d::Zero();
}

Eigen::Vector3d
MercatorProjection::targetImagePixelToProjectionSurfacePoint(const Eigen::Vector2d& target_image_pixel) const
{
  const double angle = target_image_pixel[0] * -angle_step_;
  Eigen::Vector3d point;
  // camera frame
  //  point.x() = cylinder_radius_ * std::cos(angle);
  //  point.y() = cylinder_radius_ * std::sin(angle);
  //  point.z() = (image_height_2_-target_image_pixel[1]) * height_step_;
  // optical frame
  point.x() = -cylinder_radius_ * std::sin(angle);
  point.y() = (target_image_pixel[1] - image_height_2_) * height_step_;
  point.z() = cylinder_radius_ * std::cos(angle);

  return point;
}

bool MercatorProjection::loadProjectionParameters()
{
  addReconfigurableParameter(
      "cylinder_radius", cylinder_radius_, "Radius of the cylinder used as the projection surface",
      hector::ReconfigurableParameterOptions<double>().onValidate([](const auto& value) { return value > 0; }));

  return true;
}

void MercatorProjection::onParametersChanged()
{
  const double vertical_fov_rad = 2 * M_PI * static_cast<double>(imageHeight()) / static_cast<double>(imageWidth());
  RCLCPP_INFO_STREAM(node_->get_logger(), "Vertical FOV: " << vertical_fov_rad * 180 / M_PI);
  const double cylinder_height = 2 * cylinder_radius_ * std::tan(vertical_fov_rad / 2.0);
  height_step_ = cylinder_height / static_cast<double>(imageHeight());
  angle_step_ = 2 * M_PI / static_cast<double>(imageWidth());
  image_height_2_ = static_cast<double>(imageHeight()) / 2.0;
}

}  // namespace image_projection_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(image_projection_plugins::MercatorProjection, image_projection_plugin_interface::ProjectionBase)

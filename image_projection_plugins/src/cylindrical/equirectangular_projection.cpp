#include <image_projection_plugins/cylindrical/equirectangular_projection.h>

namespace image_projection_plugins {

Eigen::Vector2d EquirectangularProjection::projectionSurfacePointToTargetImagePixel(const Eigen::Vector3d& point) const
{
  // TODO implement
  RCLCPP_ERROR(node_->get_logger(), "Not implemented");
  return Eigen::Vector2d::Zero();
}

Eigen::Vector3d
EquirectangularProjection::targetImagePixelToProjectionSurfacePoint(const Eigen::Vector2d& target_image_pixel) const
{
  const double angle_long = target_image_pixel(0) * -angle_step_;
  const double angle_lat = target_image_pixel(1) * angle_step_ - vertical_fov_rad_2_;
  Eigen::Vector3d point;
  // optical frame
  point.x() = -cylinder_radius_ * std::sin(angle_long);
  point.y() = cylinder_radius_ * std::tan(angle_lat);
  point.z() = cylinder_radius_ * std::cos(angle_long);
  return point;
}

bool EquirectangularProjection::loadProjectionParameters()
{
  addReconfigurableParameter(
      "cylinder_radius", cylinder_radius_, "Radius of the cylinder used as the projection surface",
      hector::ParameterOptions<double>().onValidate([](const auto& value) { return value > 0; }));

  return true;
}

void EquirectangularProjection::onParametersChanged()
{
  const double vertical_fov_rad = 2 * M_PI * static_cast<double>(imageHeight()) / static_cast<double>(imageWidth());
  RCLCPP_INFO_STREAM(node_->get_logger(), "Vertical FOV: " << vertical_fov_rad * 180 / M_PI);
  angle_step_ = vertical_fov_rad / static_cast<double>(imageHeight());
  vertical_fov_rad_2_ = vertical_fov_rad / 2.0;
}

}  // namespace image_projection_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(image_projection_plugins::EquirectangularProjection,
                       image_projection_plugin_interface::ProjectionBase);

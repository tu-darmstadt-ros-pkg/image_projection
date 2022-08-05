#include <image_projection_plugins/cylindrical/equirectangular_projection.h>
#include <pluginlib/class_list_macros.h>

namespace image_projection_plugins {

Eigen::Vector2d EquirectangularProjection::projectionSurfacePointToTargetImagePixel(const Eigen::Vector3d& point) const
{
  // TODO implement
  ROS_WARN_STREAM("Not implemented");
  return Eigen::Vector2d::Zero();
}

Eigen::Vector3d EquirectangularProjection::targetImagePixelToProjectionSurfacePoint(const Eigen::Vector2d& target_image_pixel) const
{
  double angle_long = target_image_pixel[0] * -angle_long_step_;
  double angle_lat = target_image_pixel(1) * angle_lat_step_ - vertical_fov_rad_2_;
  Eigen::Vector3d point;
  // optical frame
  point.x() = -cylinder_radius_ * std::sin(angle_long);
  point.y() = cylinder_radius_ * std::tan(angle_lat);
  point.z() = cylinder_radius_ * std::cos(angle_long);
  return point;
}

bool EquirectangularProjection::loadProjectionParametersFromNamespace(const ros::NodeHandle& nh)
{
  registerParameterFromNamespace(nh, "cylinder_radius", 1.0, "Cylinder radius", 0, 10);
  registerParameterFromNamespace(nh, "vertical_fov", 90, "Vertical field of view in degree", 0, 179);
  parametersChanged();

  return true;
}
void EquirectangularProjection::parametersChanged()
{
  cylinder_radius_ = getParameter("cylinder_radius");
  double vertical_fov_rad = getParameter("vertical_fov") * M_PI / 180;
  angle_lat_step_ = vertical_fov_rad / static_cast<double>(imageHeight());
  angle_long_step_ = 2*M_PI / static_cast<double>(imageWidth());
  vertical_fov_rad_2_ = vertical_fov_rad / 2.0;
}

}

PLUGINLIB_EXPORT_CLASS(image_projection_plugins::EquirectangularProjection, image_projection_plugin_interface::ProjectionBase)

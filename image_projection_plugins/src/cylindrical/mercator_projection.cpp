#include <image_projection_plugins/cylindrical/mercator_projection.h>
#include <pluginlib/class_list_macros.h>

namespace image_projection_plugins {

Eigen::Vector2d MercatorProjection::projectionSurfacePointToTargetImagePixel(const Eigen::Vector3d& point) const
{
  // TODO implement
  ROS_WARN_STREAM("Not implemented");
  return Eigen::Vector2d::Zero();
}

Eigen::Vector3d MercatorProjection::targetImagePixelToProjectionSurfacePoint(const Eigen::Vector2d& target_image_pixel) const
{
  double angle = target_image_pixel[0] * -angle_step_;
  Eigen::Vector3d point;
  point.x() = cylinder_radius_ * std::cos(angle);
  point.y() = cylinder_radius_ * std::sin(angle);
  point.z() = (image_height_2_-target_image_pixel[1]) * height_step_;

  return point;
}

bool MercatorProjection::loadProjectionParametersFromNamespace(const ros::NodeHandle& nh)
{
  registerParameterFromNamespace(nh, "cylinder_radius", 1.0, "Cylinder radius", 0, 10);
  registerParameterFromNamespace(nh, "vertical_fov", 90, "Vertical field of view in degree", 0, 179);
  parametersChanged();

  return true;
}

void MercatorProjection::parametersChanged()
{
  cylinder_radius_ = getParameter("cylinder_radius");
  double vertical_fov_rad = getParameter("vertical_fov") * M_PI / 180;
  double cylinder_height = 2 * cylinder_radius_ * std::tan(vertical_fov_rad / 2.0);
  height_step_ = cylinder_height / static_cast<double>(imageHeight());
  angle_step_ = 2*M_PI / static_cast<double>(imageWidth());
  image_height_2_ = imageHeight()/2;
}

}

PLUGINLIB_EXPORT_CLASS(image_projection_plugins::MercatorProjection, image_projection_plugin_interface::ProjectionBase)

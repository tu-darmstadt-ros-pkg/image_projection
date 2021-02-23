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
  // TODO implement
  ROS_WARN_STREAM("Not implemented");
  return Eigen::Vector3d::Zero();
}

bool EquirectangularProjection::loadProjectionParametersFromNamespace(const ros::NodeHandle& nh)
{
  // TODO implement
  return true;
}

}

PLUGINLIB_EXPORT_CLASS(image_projection_plugins::EquirectangularProjection, image_projection_plugin_interface::ProjectionBase)

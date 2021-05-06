#include <image_projection_plugins/ideal_fisheye_projection.h>
#include <pluginlib/class_list_macros.h>

namespace image_projection_plugins {

bool IdealFisheyeProjection::initialize(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
{
  nh_ = nh;
  pnh_ = pnh;

  return true;
}

Eigen::Vector2d IdealFisheyeProjection::projectionSurfacePointToTargetImagePixel(const Eigen::Vector3d& point) const
{
  // TODO implement
  ROS_WARN_STREAM("IdealFisheyeProjection::projectionSurfacePointToTargetImagePixel not implemented");
  return Eigen::Vector2d::Zero();
}

Eigen::Vector3d IdealFisheyeProjection::targetImagePixelToProjectionSurfacePoint(const Eigen::Vector2d& target_image_pixel) const
{
  // Transform to polar coordinates
  double p_x = target_image_pixel[0] - image_width_2_;
  double p_y = (image_height_2_ - target_image_pixel[1]);
  double long_angle = atan2(p_y, p_x);
  double r = sqrt(p_x * p_x + p_y * p_y);
  double lat_angle = r * angle_step_;
  if (lat_angle > fov_rad_2_) {
    return Eigen::Vector3d::Zero(); // TODO return invalid value
  }

  // Map to 3D sphere
  Eigen::Vector3d point;
  // optical frame
  point.x() = sphere_radius_ * sin(lat_angle) * cos(long_angle);
  point.y() = -sphere_radius_ * sin(lat_angle) * sin(long_angle);
  point.z() = sphere_radius_ * cos(lat_angle);

  // camera frame
//  point.x() = sphere_radius_ * cos(lat_angle);
//  point.y() = sphere_radius_ * sin(lat_angle) * cos(long_angle);
//  point.z() = sphere_radius_ * sin(lat_angle) * sin(long_angle);

  return point;
}

bool IdealFisheyeProjection::loadProjectionParametersFromNamespace(const ros::NodeHandle& nh)
{
  registerParameterFromNamespace(nh, "sphere_radius", 1, "Radius of the (virtual) fisheye sphere (in m)", 0, 10);
  registerParameterFromNamespace(nh, "fov", 180, "Fisheye horizontal and vertical field of view (in deg)", 1, 720);
  parametersChanged();
  return true;
}

void IdealFisheyeProjection::parametersChanged()
{
  fov_rad_ = getParameter("fov") * M_PI / 180;
  sphere_radius_ = getParameter("sphere_radius");
  image_width_2_ = static_cast<double>(imageWidth()) / 2.0;
  image_height_2_ = static_cast<double>(imageHeight()) / 2.0;
  fov_rad_2_ = fov_rad_ / 2.0;

  // angle per pixel, inv angular resolution
  if (imageWidth() < imageHeight()) {
    angle_step_ = fov_rad_ / imageWidth();
  } else {
    angle_step_ = fov_rad_ / imageHeight();
  }
}

}

PLUGINLIB_EXPORT_CLASS(image_projection_plugins::IdealFisheyeProjection, image_projection_plugin_interface::ProjectionBase)

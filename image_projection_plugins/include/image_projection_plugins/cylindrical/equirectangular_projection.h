#ifndef IMAGE_PROJECTION_PLUGINS_CYLINDRICAL_EQUIRECTANGULAR_PROJECTION_H
#define IMAGE_PROJECTION_PLUGINS_CYLINDRICAL_EQUIRECTANGULAR_PROJECTION_H

#include <image_projection_plugin_interface/projection_base.h>

namespace image_projection_plugins {

class EquirectangularProjection : public image_projection_plugin_interface::ProjectionBase {
public:
  Eigen::Vector2d projectionSurfacePointToTargetImagePixel(const Eigen::Vector3d& point) const override;
  Eigen::Vector3d targetImagePixelToProjectionSurfacePoint(const Eigen::Vector2d& target_image_pixel) const override;
protected:
  bool loadProjectionParametersFromNamespace(const ros::NodeHandle& nh) override;
  void parametersChanged() override;
private:
  double angle_step_;
  double cylinder_radius_;
  double vertical_fov_rad_2_;
};

}

#endif

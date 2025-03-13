#ifndef IMAGE_PROJECTION_PLUGINS_IDEAL_FISHEYE_PROJECTION_H
#define IMAGE_PROJECTION_PLUGINS_IDEAL_FISHEYE_PROJECTION_H

#include <image_projection_plugin_interface/projection_base.h>

namespace image_projection_plugins {

class IdealFisheyeProjection : public image_projection_plugin_interface::ProjectionBase
{
public:
  Eigen::Vector2d projectionSurfacePointToTargetImagePixel(const Eigen::Vector3d& point) const override;
  Eigen::Vector3d targetImagePixelToProjectionSurfacePoint(const Eigen::Vector2d& target_image_pixel) const override;

protected:
  bool loadProjectionParameters() override;

private:
  void onParametersChanged() override;

  // Projection parameters
  double sphere_radius_{1};
  double fov_rad_{180};

  double fov_rad_2_{0};
  double angle_step_{0};
  double image_width_2_{0};
  double image_height_2_{0};
};

}  // namespace image_projection_plugins

#endif

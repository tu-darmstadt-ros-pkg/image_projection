#ifndef IMAGE_PROJECTION_PLUGINS_CYLINDRICAL_MERCATOR_PROJECTION_H
#define IMAGE_PROJECTION_PLUGINS_CYLINDRICAL_MERCATOR_PROJECTION_H

#include <image_projection_plugin_interface/projection_base.h>

namespace image_projection_plugins {

class MercatorProjection : public image_projection_plugin_interface::ProjectionBase {
public:
  virtual Eigen::Vector2d projectionSurfacePointToTargetImagePixel(const Eigen::Vector3d& point) const override;
  virtual Eigen::Vector3d targetImagePixelToProjectionSurfacePoint(const Eigen::Vector2d& target_image_pixel) const override;
protected:
  virtual bool loadProjectionParametersFromNamespace(const ros::NodeHandle& nh) override;
  virtual void parametersChanged() override;
private:
  double height_step_;
  double angle_step_;
  double cylinder_radius_;
  double image_height_2_;
};

}

#endif

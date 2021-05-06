#ifndef IMAGE_PROJECTION_PLUGINS_IDEAL_FISHEYE_PROJECTION_H
#define IMAGE_PROJECTION_PLUGINS_IDEAL_FISHEYE_PROJECTION_H

#include <image_projection_plugin_interface/projection_base.h>

namespace image_projection_plugins {

class IdealFisheyeProjection : public image_projection_plugin_interface::ProjectionBase {
public:
  virtual bool initialize(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) override;
  virtual Eigen::Vector2d projectionSurfacePointToTargetImagePixel(const Eigen::Vector3d& point) const override;
  virtual Eigen::Vector3d targetImagePixelToProjectionSurfacePoint(const Eigen::Vector2d& target_image_pixel) const override;
protected:
  virtual bool loadProjectionParametersFromNamespace(const ros::NodeHandle& nh) override;
private:
  virtual void parametersChanged() override;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Projection parameters
  double sphere_radius_;
  double fov_rad_;
  double fov_rad_2_;
  double angle_step_;
  double image_width_2_;
  double image_height_2_;
};

}

#endif

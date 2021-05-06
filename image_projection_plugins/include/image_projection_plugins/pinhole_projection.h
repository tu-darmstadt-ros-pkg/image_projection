#ifndef IMAGE_PROJECTION_PLUGINS_PINHOLE_PROJECTION_H
#define IMAGE_PROJECTION_PLUGINS_PINHOLE_PROJECTION_H

#include <image_projection_plugin_interface/projection_base.h>
#include <sensor_msgs/CameraInfo.h>

namespace image_projection_plugins {

class PinholeProjection : public image_projection_plugin_interface::ProjectionBase {
public:
  virtual bool initialize(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) override;
  virtual Eigen::Vector2d projectionSurfacePointToTargetImagePixel(const Eigen::Vector3d& point) const override;
  virtual Eigen::Vector3d targetImagePixelToProjectionSurfacePoint(const Eigen::Vector2d& target_image_pixel) const override;
protected:
  virtual bool loadProjectionParametersFromNamespace(const ros::NodeHandle& nh) override;
private:
  virtual void parametersChanged() override;
  void publishCameraInfo();
  sensor_msgs::CameraInfo parametersToCameraInfo() const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher camera_info_pub_;

  // Projection parameters
  double m_per_pixel_;
  double focal_length_;
  double image_width_2_;
  double image_height_2_;

  std::string virtual_sensor_optical_frame_;
};

}

#endif

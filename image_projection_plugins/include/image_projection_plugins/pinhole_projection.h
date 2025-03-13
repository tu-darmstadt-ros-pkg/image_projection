#ifndef IMAGE_PROJECTION_PLUGINS_PINHOLE_PROJECTION_H
#define IMAGE_PROJECTION_PLUGINS_PINHOLE_PROJECTION_H

#include <image_projection_plugin_interface/projection_base.h>
#include <sensor_msgs/msg/camera_info.hpp>

namespace image_projection_plugins {

class PinholeProjection : public image_projection_plugin_interface::ProjectionBase
{
public:
  bool initialize(const rclcpp::Node::SharedPtr& node, const std::string& name) override;
  Eigen::Vector2d projectionSurfacePointToTargetImagePixel(const Eigen::Vector3d& point) const override;
  Eigen::Vector3d targetImagePixelToProjectionSurfacePoint(const Eigen::Vector2d& target_image_pixel) const override;

protected:
  bool loadProjectionParameters() override;

private:
  void onParametersChanged() override;
  void publishCameraInfo() const;
  sensor_msgs::msg::CameraInfo parametersToCameraInfo() const;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  // Projection parameters
  double focal_length_{1};
  double horizontal_fov{90};

  double m_per_pixel_{0};
  double image_width_2_{0};
  double image_height_2_{0};

  std::string virtual_sensor_optical_frame_;
};

}  // namespace image_projection_plugins

#endif

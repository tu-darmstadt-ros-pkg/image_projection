#ifndef IMAGE_PROJECTION_PLUGIN_INTERFACE_PROJECTIONS_PROJECTION_BASE_H
#define IMAGE_PROJECTION_PLUGIN_INTERFACE_PROJECTIONS_PROJECTION_BASE_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>

namespace image_projection_plugin_interface {
class ProjectionBase
{
public:
  virtual ~ProjectionBase();
  virtual bool initialize(const rclcpp::Node::SharedPtr& node, const std::string& name);

  bool loadParameters();

  /**
   * Notify users of this plugin that parameters have changed that affect the projection mapping, e.g. projection
   * parameters of image dimensions
   * @return
   */
  [[nodiscard]] bool mappingChanged() const;
  void unsetMappingChanged();

  [[nodiscard]] virtual Eigen::Vector2d projectionSurfacePointToTargetImagePixel(const Eigen::Vector3d& point) const = 0;
  [[nodiscard]] virtual Eigen::Vector3d
  targetImagePixelToProjectionSurfacePoint(const Eigen::Vector2d& target_image_pixel) const = 0;

  void setImageWidth(int image_width);
  [[nodiscard]] int imageWidth() const;

  void setImageHeight(int image_height);
  [[nodiscard]] int imageHeight() const;

protected:
  virtual bool loadProjectionParameters() = 0;

  template <typename ParameterT>
  void addReconfigurableParameter(const std::string& name, ParameterT& param, const std::string& description,
                                  const hector::ReconfigurableParameterOptions<ParameterT>& options = {})
  {
    param_subscriptions_.push_back(
        hector::createReconfigurableParameter(node_, name_ + "." + name, param, description, options));
  }

  /**
   * Called to notify plugins that parameters have changed
   */
  virtual void onParametersChanged();

  rclcpp::Node::SharedPtr node_;

private:
  void parameterUpdateCallback(const std::vector<rclcpp::Parameter>& parameters);
  bool loadBaseParameters();

  bool mapping_changed_{true};

  // Parameters
  std::string name_;  // Name of this plugin. Used as parameter namespace
  int image_width_{0};
  int image_height_{0};

  std::vector<hector::ReconfigurableParameterSubscription> param_subscriptions_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameter_cb_handle_;
};
}  // namespace image_projection_plugin_interface

#endif

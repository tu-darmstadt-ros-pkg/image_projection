#include <image_projection_plugin_interface/projection_base.h>

#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>

namespace image_projection_plugin_interface {

ProjectionBase::~ProjectionBase() = default;

bool ProjectionBase::initialize(const rclcpp::Node::SharedPtr& node, const std::string& name)
{
  // TODO try with sub_nodes?
  node_ = node;
  name_ = name;

  parameter_cb_handle_ = node_->add_post_set_parameters_callback(
      std::bind(&ProjectionBase::parameterUpdateCallback, this, std::placeholders::_1));
  return true;
}

bool ProjectionBase::loadParameters()
{
  bool success = true;
  success &= loadBaseParameters();
  success &= loadProjectionParameters();
  mapping_changed_ = true;
  onParametersChanged();
  return success;
}

bool ProjectionBase::mappingChanged() const
{
  return mapping_changed_;
}

void ProjectionBase::unsetMappingChanged()
{
  mapping_changed_ = false;
}

int ProjectionBase::imageWidth() const
{
  return image_width_;
}

int ProjectionBase::imageHeight() const
{
  return image_height_;
}

void ProjectionBase::onParametersChanged() {}

void ProjectionBase::setImageWidth(const int image_width)
{
  image_width_ = image_width;
}

void ProjectionBase::setImageHeight(int image_height)
{
  image_height_ = image_height;
}

void ProjectionBase::parameterUpdateCallback(const std::vector<rclcpp::Parameter>& parameters)
{
  bool parameters_changed = false;
  for (const auto& parameter : parameters) {
    // Check if parameter starts with the name of this plugin
    if (parameter.get_name().rfind(name_, 0) == 0) {
      parameters_changed = true;
      break;
    }
  }
  // Notify plugins and external users
  if (parameters_changed) {
    mapping_changed_ = true;
    onParametersChanged();
  }
}

bool ProjectionBase::loadBaseParameters()
{
  bool success = true;  // TODO make parameters mandatory
  addReconfigurableParameter(
      "image_width", image_width_, "Output image width",
      hector::ParameterOptions<int>().onValidate([](const auto& value) { return value > 0; }));

  addReconfigurableParameter(
      "image_height", image_height_, "Output image height",
      hector::ParameterOptions<int>().onValidate([](const auto& value) { return value > 0; }));

  return success;
}

}  // namespace image_projection_plugin_interface

#include <image_projection_plugin_interface/projection_base.h>

#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>

namespace image_projection_plugin_interface {

ProjectionBase::~ProjectionBase() {}

bool ProjectionBase::initialize(const rclcpp::Node::SharedPtr& node, const std::string& name)
{
  node_ = node;
  name_ = name;
  return true;
}

bool ProjectionBase::loadParameters()
{
  bool success = true;
  success &= loadBaseParameters();
  success &= loadProjectionParameters();
  mapping_changed_ = true;
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

void ProjectionBase::parametersChanged()
{}

void ProjectionBase::setImageWidth(const int image_width)
{
  image_width_ = image_width;
}

void ProjectionBase::setImageHeight(int image_height)
{
  image_height_ = image_height;
}

bool ProjectionBase::loadBaseParameters()
{
  bool success = true; //TODO make parameters mandatory
  addReconfigurableParameter("image_width", image_width_, "Output image width",
    hector::ReconfigurableParameterOptions<int>()
    .onValidate([]( const auto &value ) {
      return value > 0;
    }));

  addReconfigurableParameter("image_height", image_height_, "Output image height",
  hector::ReconfigurableParameterOptions<int>()
  .onValidate([]( const auto &value ) {
    return value > 0;
  }));

  return success;
}

}

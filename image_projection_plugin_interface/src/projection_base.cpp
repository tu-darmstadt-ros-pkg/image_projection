#include <image_projection_plugin_interface/projection_base.h>

#include <image_projection_plugin_interface/utils.h>

namespace image_projection_plugin_interface {

ProjectionBase::~ProjectionBase() {}

bool ProjectionBase::initialize(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
{
  return true;
}

bool ProjectionBase::loadParametersFromNamespace(const ros::NodeHandle& nh)
{
  reconfigure_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh);
  bool success = true;
  success &= loadBaseParametersFromNamespace(nh);
  success &= loadProjectionParametersFromNamespace(nh);
  mapping_changed_ = true;
  reconfigure_->publishServicesTopics();
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

void ProjectionBase::registerParameter(const std::string& name, double value, const std::string& description, double min, double max)
{
  config_[name] = value;
  reconfigure_->registerVariable<double>(name, value, boost::bind(&ProjectionBase::reconfigureCallback, this, name, _1), description, min, max);
}

void ProjectionBase::registerParameterFromNamespace(const ros::NodeHandle& nh, const std::string& name, double default_value, const std::string& description, double min, double max)
{
  double value;
  nh.param(name, value, default_value);
  registerParameter(name, value, description, min, max);
}

double ProjectionBase::getParameter(const std::string& name) const
{
  try {
    return config_.at(name);
  } catch (const std::out_of_range&) {
    ROS_ERROR_STREAM("Failed to load parameter '" << name << "'");
    return 0.0;
  }
}

void ProjectionBase::parametersChanged()
{}

void ProjectionBase::setImageWidth(int image_width)
{
  image_width_ = image_width;
}

void ProjectionBase::setImageHeight(int image_height)
{
  image_height_ = image_height;
}

bool ProjectionBase::loadBaseParametersFromNamespace(const ros::NodeHandle& nh)
{
  bool success = true;
  success &= loadMandatoryParameter(nh, "image_width", image_width_);
  reconfigure_->registerVariable<int>("image_width", image_width_, boost::bind(&ProjectionBase::imageDimensionsReconfigureCallback, this, &image_width_, _1), "Image width", 0, 10000);
  success &= loadMandatoryParameter(nh, "image_height", image_height_);
  reconfigure_->registerVariable<int>("image_height", image_height_, boost::bind(&ProjectionBase::imageDimensionsReconfigureCallback, this, &image_height_, _1), "Image height", 0, 10000);
  return success;
}

void ProjectionBase::reconfigureCallback(const std::string& name, double value)
{
  ROS_INFO_STREAM("Reconfigure request: " << name << ": " << value);
  config_[name] = value;
  mapping_changed_ = true;
  parametersChanged();
}

void ProjectionBase::imageDimensionsReconfigureCallback(int* field, int value)
{
  *field = value;
  ROS_INFO_STREAM("Reconfigure request: Image dimensions [" << imageWidth() << "x" << imageHeight() << "]");
  mapping_changed_ = true;
  parametersChanged();
}

}

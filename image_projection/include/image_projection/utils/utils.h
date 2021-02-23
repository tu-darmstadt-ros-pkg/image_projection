#ifndef IMAGE_PROJECTION_UTILS_UTILS_H
#define IMAGE_PROJECTION_UTILS_UTILS_H

#include <ros/ros.h>

namespace image_projection {

template <typename T>
bool loadMandatoryParameter(const ros::NodeHandle& nh, std::string parameter_name, T& value_out) {
  if (!nh.getParam(parameter_name, value_out)) {
    ROS_ERROR_STREAM("Failed to load parameter '" << nh.getNamespace() << "/" << parameter_name << "'");
    return false;
  }
  return true;
}

}

#endif

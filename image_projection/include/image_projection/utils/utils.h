#ifndef IMAGE_PROJECTION_UTILS_UTILS_H
#define IMAGE_PROJECTION_UTILS_UTILS_H

#include <rclcpp/rclcpp.hpp>

namespace image_projection {

template <typename T>
bool loadMandatoryParameter(const rclcpp::Node::SharedPtr node, std::string parameter_name, T& value_out) {
  rclcpp::Parameter parameter;
  node->declare_parameter<T>(parameter_name);
  node->get_parameter(parameter_name, value_out);
  /*if (node->get_parameter(parameter_name, value_out)) {
    value_out = parameter.get_value<T>();
    RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to load parameter '" << node->get_namespace() << "/" << parameter_name << "'");
    return false;
  }*/
  return true;
}

}

#endif

#include <image_projection/image_projection_server.h>

namespace image_projection {

ImageProjectionServer::ImageProjectionServer(const rclcpp::NodeOptions& options)
    : rclcpp::Node("image_projection_server", options)
{}
}  // namespace image_projection
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_projection::ImageProjectionServer)

#include <image_projection/periodic_image_projection_node.h>

namespace image_projection {

PeriodicImageProjectionNode::PeriodicImageProjectionNode(const rclcpp::NodeOptions& options) : node_(std::make_shared<rclcpp::Node>("periodic_image_projection_node", options)) {
  // Create PeriodicImageProjection with shared node handle
  periodic_image_projection_ = std::make_shared<PeriodicImageProjection>(node_);

  if (!periodic_image_projection_->init()) {
    RCLCPP_ERROR(node_->get_logger(), "Initialization of PeriodicImageProjection failed. Exiting.");
    rclcpp::shutdown();
    return;
  }

  // Declare or get update rate
  double update_rate = node_->declare_parameter("update_rate", 1.0);
  using namespace std::chrono_literals;
  auto period = std::chrono::duration<double>(1.0 / update_rate);

  timer_ = node_->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&PeriodicImageProjectionNode::timerCb, this));
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr PeriodicImageProjectionNode::get_node_base_interface() const
{
  return this->node_->get_node_base_interface();
}

void PeriodicImageProjectionNode::timerCb() {
  periodic_image_projection_->projectAndPublishLatestImages();
}

}  // namespace image_projection

// Register as composable
RCLCPP_COMPONENTS_REGISTER_NODE(image_projection::PeriodicImageProjectionNode)

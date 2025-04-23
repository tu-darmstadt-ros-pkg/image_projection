#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <image_projection/periodic_image_projection.h>

namespace image_projection {
class PeriodicImageProjectionNode {
public:
  explicit PeriodicImageProjectionNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

private:
  void timerCb();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<PeriodicImageProjection> periodic_image_projection_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace image_projection

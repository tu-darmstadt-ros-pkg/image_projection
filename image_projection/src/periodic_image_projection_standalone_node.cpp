#include <image_projection/periodic_image_projection_node.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto projection = std::make_shared<image_projection::PeriodicImageProjectionNode>(rclcpp::NodeOptions{});
  auto node = projection->get_node_base_interface();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

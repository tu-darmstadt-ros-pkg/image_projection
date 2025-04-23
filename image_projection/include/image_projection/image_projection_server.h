#ifndef IMAGE_PROJECTION_IMAGE_PROJECTION_SERVER_H
#define IMAGE_PROJECTION_IMAGE_PROJECTION_SERVER_H

#include <rclcpp/rclcpp.hpp>

namespace image_projection {

class ImageProjectionServer : public rclcpp::Node {
public:
  ImageProjectionServer(const rclcpp::NodeOptions& options);
private:

  //rclcpp::Service<>::SharedPtr projection_srv_;

};

}

#endif

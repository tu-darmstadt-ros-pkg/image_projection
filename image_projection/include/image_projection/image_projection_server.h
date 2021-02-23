#ifndef IMAGE_PROJECTION_IMAGE_PROJECTION_SERVER_H
#define IMAGE_PROJECTION_IMAGE_PROJECTION_SERVER_H

#include <ros/ros.h>

namespace image_projection {

class ImageProjectionServer {
public:
  ImageProjectionServer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::ServiceServer projection_srv_;

};

}

#endif

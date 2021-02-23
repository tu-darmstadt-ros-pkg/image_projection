#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <image_projection/periodic_image_projection.h>

namespace image_projection {

class PeriodicImageProjectionNodelet : public nodelet::Nodelet {
  void onInit() override {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& pnh = getPrivateNodeHandle();
    periodic_image_projection_ = std::make_shared<PeriodicImageProjection>(nh, pnh);
    if (!periodic_image_projection_->init()) {
      exit(0);
    }

    double update_rate = pnh.param("update_rate", 1.0);
    timer_ = nh.createTimer(ros::Duration(1.0/update_rate), &PeriodicImageProjectionNodelet::timerCb, this, false);
  }

  void timerCb(const ros::TimerEvent&) {
    periodic_image_projection_->projectAndPublishLatestImages();
  }

  std::shared_ptr<PeriodicImageProjection> periodic_image_projection_;
  ros::Timer timer_;
};
}

PLUGINLIB_EXPORT_CLASS(image_projection::PeriodicImageProjectionNodelet, nodelet::Nodelet);

#ifndef IMAGE_PROJECTION_H
#define IMAGE_PROJECTION_H

#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include <kalibr_camera_loader/camera_loader.h>

#include <tf2_ros/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>

#include <pluginlib/class_loader.h>
#include <image_projection_plugin_interface/projection_base.h>

namespace image_projection {

typedef std::unordered_map<std::string, std::pair<cv::UMat, cv::UMat>> PixelMapping;
typedef std::shared_ptr<image_projection_plugin_interface::ProjectionBase> ProjectionPtr;
typedef pluginlib::ClassLoader<image_projection_plugin_interface::ProjectionBase> ProjectionClassLoader;

class ImageProjection {
public:
  ImageProjection(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  ~ImageProjection();

  ProjectionPtr loadProjectionPlugin(const std::string& projection_name);
  PixelMapping createMapping(const ProjectionPtr& projection, const std::string& base_frame, const ros::Time& stamp=ros::Time::now(),
                             const Eigen::Isometry3d& sensor_pose=Eigen::Isometry3d::Identity()) const;
  std::map<std::string, cv_bridge::CvImageConstPtr> getLatestImages(ros::Time& stamp, std::string& encoding) const;
  bool projectImages(const std::map<std::string, cv_bridge::CvImageConstPtr>& images, const PixelMapping& pixel_mapping, cv::UMat& projection) const;
  bool projectLatestImages(const PixelMapping& pixel_mapping, cv::UMat& projection, ros::Time& stamp, std::string& encoding) const;

  kalibr_image_geometry::CameraLoader& getCameraLoader();
private:
  ros::NodeHandle nh_;

  ProjectionClassLoader projection_loader_;
  kalibr_image_geometry::CameraLoader camera_loader_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool use_opencl_;

  std::string save_folder_;
};

}

#endif

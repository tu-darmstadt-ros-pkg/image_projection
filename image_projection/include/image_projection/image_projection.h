#ifndef IMAGE_PROJECTION_H
#define IMAGE_PROJECTION_H

#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include <extended_camera_loader/camera_loader.h>

#include <tf2_ros/transform_listener.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include <pluginlib/class_loader.hpp>
#include <image_projection_plugin_interface/projection_base.h>

namespace image_projection {

typedef std::unordered_map<std::string, std::pair<cv::UMat, cv::UMat>> PixelMapping;
typedef std::shared_ptr<image_projection_plugin_interface::ProjectionBase> ProjectionPtr;
typedef pluginlib::ClassLoader<image_projection_plugin_interface::ProjectionBase> ProjectionClassLoader;

class ImageProjection
{
public:
  using CvImageMap = std::unordered_map<std::string, cv_bridge::CvImageConstPtr>;
  explicit ImageProjection(const rclcpp::Node::SharedPtr& node);
  ~ImageProjection();

  ProjectionPtr loadProjectionPlugin(const std::string& projection_name);
  [[nodiscard]] PixelMapping createMapping(const ProjectionPtr& projection, const std::string& base_frame,
                                           const rclcpp::Time& stamp = rclcpp::Clock().now(),
                                           const Eigen::Isometry3d& sensor_pose = Eigen::Isometry3d::Identity()) const;
  CvImageMap getLatestImages(rclcpp::Time& stamp, std::string& encoding) const;
  bool projectImages(const CvImageMap& images, const PixelMapping& pixel_mapping, cv::UMat& projection) const;
  bool projectLatestImages(const PixelMapping& pixel_mapping, cv::UMat& projection, rclcpp::Time& stamp,
                           std::string& encoding) const;

  extended_image_geometry::CameraLoader& getCameraLoader();

private:
  rclcpp::Node::SharedPtr node_;

  ProjectionClassLoader projection_loader_;
  extended_image_geometry::CameraLoader camera_loader_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool use_opencl_;

  std::string save_folder_;
};

}  // namespace image_projection

#endif

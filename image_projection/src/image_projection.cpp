#include <image_projection/image_projection.h>

#include <memory>
#include <opencv2/core/ocl.hpp>

#include <image_projection/utils/timing.h>
#include <numeric>

INIT_TIMING

namespace image_projection {

ImageProjection::ImageProjection(const rclcpp::Node::SharedPtr& node)
    : node_(node),
      projection_loader_("image_projection_plugin_interface", "image_projection_plugin_interface::ProjectionBase"),
      camera_loader_(node),
      use_opencl_(true)
{

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Load parameters
  node_->declare_parameter("save_folder", std::string(""));
  node_->get_parameter("save_folder", save_folder_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "Save folder: " << save_folder_);

  // Use OpenCL
  node_->declare_parameter("use_opencl", false);
  node_->get_parameter("use_opencl", use_opencl_);
  cv::ocl::setUseOpenCL(use_opencl_);
  if (use_opencl_) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "OpenCL: " << cv::ocl::haveOpenCL());
    std::vector<cv::ocl::PlatformInfo> platform_info;
    cv::ocl::getPlatfomsInfo(platform_info);
    for (const auto& i : platform_info) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "\tName: " << i.name() << std::endl
                                                         << "\tVendor: " << i.vendor() << std::endl
                                                         << "\tVersion: " << i.version() << std::endl
                                                         << "\tDevice Number: " << i.deviceNumber() << std::endl);
    }
  }
}

ImageProjection::~ImageProjection()
{
  timing::Timing::printTimeInfos();
}

ProjectionPtr ImageProjection::loadProjectionPlugin(const std::string& projection_name)
{
  ProjectionPtr projection;
  try {
    image_projection_plugin_interface::ProjectionBase* projection_ptr =
        projection_loader_.createUnmanagedInstance(projection_name);
    projection.reset(projection_ptr,
                     std::bind(&ProjectionClassLoader::unloadLibraryForClass, &projection_loader_, projection_name));
  }
  catch (pluginlib::PluginlibException& ex) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "The plugin failed to load: " << ex.what());
  }
  return projection;
}

PixelMapping ImageProjection::createMapping(const ProjectionPtr& projection, const std::string& base_frame,
                                            const rclcpp::Time& stamp, const Eigen::Isometry3d& sensor_pose) const
{
  PixelMapping pixel_mapping_umat;
  if (!projection) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "ProjectionPtr is none");
    return pixel_mapping_umat;
  }
  START_TIMING("mapping")
  RCLCPP_INFO_STREAM(node_->get_logger(), "Creating mapping");
  cv::Mat distance(projection->imageHeight(), projection->imageWidth(), CV_32F, extended_image_geometry::INVALID);
  std::unordered_map<std::string, std::pair<cv::Mat, cv::Mat>> pixel_mapping;
  for (const extended_image_geometry::CameraPtr& cam : camera_loader_.cameras()) {
    // Get transform to camera frame
    std::string cam_frame_id;
    if (!cam->model().cameraInfo()->frame_id.empty()) {
      cam_frame_id = cam->model().cameraInfo()->frame_id;
    } else {
      cam_frame_id = cam->model().cameraInfo()->header.frame_id;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "Waiting for transformation from '" << base_frame << "' to '" << cam_frame_id << "' ..");
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(cam_frame_id, base_frame, stamp, rclcpp::Duration(1, 0));
    }
    catch (const tf2::TransformException& e) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "LookupTransform failed. Reason: " << e.what());
      return {};
    }
    Eigen::Isometry3d cam_to_world = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d sensor_to_world = cam_to_world * sensor_pose;

    // Iterate over every pixel of projection image
    cv::Mat mapping_x(projection->imageHeight(), projection->imageWidth(), CV_32F, -1);
    cv::Mat mapping_y(projection->imageHeight(), projection->imageWidth(), CV_32F, -1);
    for (int row = 0; row < projection->imageHeight(); row++) {
      for (int col = 0; col < projection->imageWidth(); col++) {
        // transform cloud to cam frame
        // Compute ray that goes through pixel of projection shape
        Eigen::Vector3d world_ray = projection->targetImagePixelToProjectionSurfacePoint(Eigen::Vector2d(col, row));
        // Transform to cam frame
        Eigen::Vector3d cam_ray = sensor_to_world * world_ray;

        // find corresponding cam pixel
        Eigen::Vector2d cam_pixel;
        bool success = cam->model().worldToPixel(cam_ray, cam_pixel);
        if (success) {
          // Check distance to cam center
          auto new_distance = static_cast<float>(cam->model().distanceFromCenter(cam_pixel));
          // if distance is smaller, take pixel of this camera instead
          if (new_distance < distance.at<float>(row, col)) {
            distance.at<float>(row, col) = new_distance;

            // disable mapping of other cams onto this pixel
            for (auto& [_, pm] : pixel_mapping) {
              pm.first.at<float>(row, col) = -1;
              pm.second.at<float>(row, col) = -1;
            }
            // Set mapping of this cam for this pixel
            mapping_x.at<float>(row, col) = static_cast<float>(cam_pixel(0));
            mapping_y.at<float>(row, col) = static_cast<float>(cam_pixel(1));
          }
        }
      }
    }

    //    cv::Mat converted_1, converted_2;
    //    cv::convertMaps(mapping_x, mapping_y, converted_1, converted_2, CV_16SC2); // Converted maps lead to blurry
    //    projections pixel_mapping[cam.getName()] = std::make_pair(converted_1, converted_2);
    pixel_mapping[cam->getName()] = std::make_pair(mapping_x, mapping_y);
  }
  // Move to GPU
  for (const auto& [key, pm] : pixel_mapping) {
    cv::UMat mapping_x_umat;
    cv::UMat mapping_y_umat;
    mapping_x_umat = pm.first.getUMat(cv::ACCESS_READ);
    mapping_y_umat = pm.second.getUMat(cv::ACCESS_READ);
    pixel_mapping_umat[key] = std::make_pair(mapping_x_umat, mapping_y_umat);
  }

  STOP_TIMING_AVG
  RCLCPP_INFO_STREAM(node_->get_logger(), "Mapping finished.");
  return pixel_mapping_umat;
}

ImageProjection::CvImageMap ImageProjection::getLatestImages(rclcpp::Time& stamp, std::string& encoding) const
{
  // Retrieve images from all cams
  // TODO: lock image retrieval
  std::unordered_map<std::string, std::shared_ptr<sensor_msgs::msg::Image const>> images;
  std::vector<int64_t> stamps;
  for (const extended_image_geometry::CameraPtr& cam : camera_loader_.cameras()) {
    const std::shared_ptr<sensor_msgs::msg::Image const> image = cam->getLastImage();
    if (image) {
      stamps.push_back(rclcpp::Time(image->header.stamp).nanoseconds());
      images[cam->getName()] = image;
    }
  }

  std::unordered_map<std::string, cv_bridge::CvImageConstPtr> cv_images;
  // Return empty map if no image has been received
  if (images.empty()) {
    return cv_images;
  }

  // If no encoding is given, take encoding of first image
  if (encoding.empty()) {
    encoding = images.begin()->second->encoding;
  }

  // Convert to cv
  for (const auto& [key, image] : images) {
    if (image->encoding != encoding) {
      RCLCPP_WARN_STREAM_ONCE(node_->get_logger(),
                              "Image of camera '" << key << "' does not match desired encoding '" << encoding
                                                  << "'. Image data is copied. This warning is printed only once.");
    }
    try {
      cv_images[key] = cv_bridge::toCvShare(image, encoding);
    }
    catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "CV Bridge conversion failed: " << e.what());
    }
  }

  // Compute average stamp
  std::transform(begin(stamps), end(stamps), begin(stamps),
                 [size = stamps.size()](const int64_t& x) { return x / size; });
  stamp = rclcpp::Time(std::accumulate(begin(stamps), end(stamps), 0L));
  return cv_images;
}

bool ImageProjection::projectImages(const CvImageMap& images, const PixelMapping& pixel_mapping,
                                    cv::UMat& projection) const
{
  if (images.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
                                "No cam image received yet. Can't project. This message is throttled.");
    return false;
  }
  START_TIMING("project")
  for (const auto& [cam_name, cv_image] : images) {
    try {
      const auto& [mapping_src, mapping_dest] = pixel_mapping.at(cam_name);
      cv::UMat image = cv_image->image.getUMat(cv::ACCESS_READ);
      cv::remap(image, projection, mapping_src, mapping_dest, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
    }
    catch (std::out_of_range&) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "No mapping available for camera '" << cam_name << "'.");
      return false;
      STOP_TIMING_AVG
    }
  }

  if (!save_folder_.empty()) {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S");
    std::string time_str = oss.str();

    RCLCPP_INFO_STREAM(node_->get_logger(), "Saving image to: " << save_folder_ << "/projection_" << time_str << ".jpg");
    cv::UMat bgr_img;
    cv::cvtColor(projection, bgr_img, cv::COLOR_RGB2BGR);
    cv::imwrite(save_folder_ + "/projection_" + time_str + ".jpg", bgr_img);
  }
  STOP_TIMING_AVG

  return true;
}

bool ImageProjection::projectLatestImages(const PixelMapping& pixel_mapping, cv::UMat& projection, rclcpp::Time& stamp,
                                          std::string& encoding) const
{
  auto images = getLatestImages(stamp, encoding);
  return projectImages(images, pixel_mapping, projection);
}

extended_image_geometry::CameraLoader& ImageProjection::getCameraLoader()
{
  return camera_loader_;
}

}  // namespace image_projection

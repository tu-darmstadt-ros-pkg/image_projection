#include <image_projection/image_projection.h>

#include <opencv2/core/ocl.hpp>

#include <image_projection/utils/timing.h>
#include <numeric>

INIT_TIMING

namespace image_projection {

ImageProjection::ImageProjection(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), projection_loader_("image_projection_plugin_interface", "image_projection_plugin_interface::ProjectionBase"), camera_loader_(nh, pnh), use_opencl_(true) {

  tf_buffer_.reset(new tf2_ros::Buffer());
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

  // Load parameters
  pnh.param<std::string>("save_folder", save_folder_, "");
  ROS_INFO_STREAM("Save folder: " << save_folder_);

  // Use OpenCL
  pnh.param("use_opencl", use_opencl_, false);
  cv::ocl::setUseOpenCL(use_opencl_);
  if (use_opencl_) {
    ROS_INFO_STREAM("OpenCL: " << cv::ocl::haveOpenCL());
    std::vector<cv::ocl::PlatformInfo> platform_info;
    cv::ocl::getPlatfomsInfo(platform_info);
    for (const auto & i : platform_info)
    {
        ROS_INFO_STREAM(
            "\tName: " << i.name() << std::endl
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
    image_projection_plugin_interface::ProjectionBase* projection_ptr = projection_loader_.createUnmanagedInstance(projection_name);
    projection.reset(projection_ptr, std::bind(&ProjectionClassLoader::unloadLibraryForClass, &projection_loader_, projection_name));
  } catch (pluginlib::PluginlibException& ex) {
    ROS_ERROR_STREAM("The plugin failed to load: " << ex.what());
  }
  return projection;
}

PixelMapping ImageProjection::createMapping(const ProjectionPtr& projection, const std::string& base_frame, const ros::Time& stamp, const Eigen::Isometry3d& sensor_pose) const {
  PixelMapping pixel_mapping_umat;
  if (!projection) {
    ROS_ERROR_STREAM("ProjectionPtr is none");
    return pixel_mapping_umat;
  }
  START_TIMING("mapping")
  ROS_INFO_STREAM("Creating mapping");
  cv::Mat distance(projection->imageHeight(), projection->imageWidth(), CV_64F, kalibr_image_geometry::INVALID);
  std::unordered_map<std::string, std::pair<cv::Mat, cv::Mat>> pixel_mapping;
  for (const kalibr_image_geometry::CameraPtr& cam: camera_loader_.cameras()) {
    // Get transform to camera frame
    std::string cam_frame_id;
    if (!cam->model().cameraInfo().frame_id.empty()) {
      cam_frame_id = cam->model().cameraInfo().frame_id;
    } else {
      cam_frame_id = cam->model().cameraInfo().header.frame_id;
    }

    ROS_INFO_STREAM("Waiting for transformation from '" << base_frame << "' to '" << cam_frame_id << "' ..");
    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(cam_frame_id, base_frame, stamp, ros::Duration(1));
    } catch (const tf2::TransformException& e) {
      ROS_WARN_STREAM("LookupTransform failed. Reason: " << e.what());
      return PixelMapping();
    }
    Eigen::Isometry3d cam_to_world;
    tf::transformMsgToEigen(transform.transform, cam_to_world);
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
          double new_distance = cam->model().distanceFromCenter(cam_pixel);
          // if distance is smaller, take pixel of this camera instead
          if (new_distance < distance.at<double>(row, col)) {
            distance.at<double>(row, col) = new_distance;

            // disable mapping of other cams onto this pixel
            for (auto & pm : pixel_mapping) {
              pm.second.first.at<float>(row, col) = -1;
              pm.second.second.at<float>(row, col) = -1;
            }
            // Set mapping of this cam for this pixel
            mapping_x.at<float>(row, col) = static_cast<float>(cam_pixel(0));
            mapping_y.at<float>(row, col) = static_cast<float>(cam_pixel(1));
          }
        }
      }
    }

//    cv::Mat converted_1, converted_2;
//    cv::convertMaps(mapping_x, mapping_y, converted_1, converted_2, CV_16SC2); // Converted maps lead to blurry projections
//    pixel_mapping[cam.getName()] = std::make_pair(converted_1, converted_2);
    pixel_mapping[cam->getName()] = std::make_pair(mapping_x, mapping_y);
  }
  // Move to GPU
  for (const auto & pm : pixel_mapping) {
    cv::UMat mapping_x_umat, mapping_y_umat;
    mapping_x_umat = pm.second.first.getUMat(cv::ACCESS_READ);
    mapping_y_umat = pm.second.second.getUMat(cv::ACCESS_READ);
    pixel_mapping_umat[pm.first] = std::make_pair(mapping_x_umat, mapping_y_umat);
  }

  STOP_TIMING_AVG
  ROS_INFO_STREAM("Mapping finished.");
  return pixel_mapping_umat;
}

std::map<std::string, cv_bridge::CvImageConstPtr> ImageProjection::getLatestImages(ros::Time& stamp, std::string& encoding) const
{
  // Retrieve images from all cams
  // TODO: lock image retrieval
  std::map<std::string, sensor_msgs::ImageConstPtr> images;
  std::vector<uint64_t> stamps;
  for (const kalibr_image_geometry::CameraPtr& cam: camera_loader_.cameras()) {
    const sensor_msgs::ImageConstPtr& image = cam->getLastImage();
    if (image) {
      stamps.push_back(image->header.stamp.toNSec());
      images[cam->getName()] = image;
    }
  }

  std::map<std::string, cv_bridge::CvImageConstPtr> cv_images;
  // Return empty map if no image has been received
  if (images.empty()) {
    return cv_images;
  }

  // If no encoding is given, take encoding of first image
  if (encoding.empty()) {
    encoding = images.begin()->second->encoding;
  }

  // Convert to cv
  for (const auto& image: images) {
    if (image.second->encoding != encoding) {
      ROS_WARN_STREAM_ONCE("Image of camera '" << image.first << "' does not match desired encoding '" << encoding << "'. Image data is copied. This warning is printed only once.");
    }
    try
    {
      cv_images[image.first] = cv_bridge::toCvShare(image.second, encoding);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR_STREAM("CV Bridge conversion failed: " << e.what());
    }
  }

  // Compute average stamp
  std::transform(begin(stamps), end(stamps), begin(stamps), [stamps](uint64_t& x){return x/stamps.size();});
  stamp.fromNSec(std::accumulate(begin(stamps), end(stamps), 0ul));
  return cv_images;
}

bool ImageProjection::projectImages(const std::map<std::string, cv_bridge::CvImageConstPtr>& images, const PixelMapping& pixel_mapping, cv::UMat& projection) const
{
  if (images.empty()) {
    ROS_WARN_STREAM_THROTTLE(1, "No cam image received yet. Can't project. This message is throttled.");
    return false;
  }
  START_TIMING("project")
  for (const auto& kv: images) {
    const std::string& cam_name = kv.first;
    const cv_bridge::CvImageConstPtr& cv_image = kv.second;
    try {
      const PixelMapping::mapped_type& mapping_entry = pixel_mapping.at(cam_name);
      cv::UMat image = cv_image->image.getUMat(cv::ACCESS_READ);
      cv::remap(image, projection, mapping_entry.first, mapping_entry.second, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

    } catch (std::out_of_range&) {
      ROS_ERROR_STREAM("No mapping available for camera '" << cam_name << "'.");
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

    ROS_INFO_STREAM("Saving image to: " << save_folder_ << "/projection_" << time_str << ".jpg");
    cv::UMat bgr_img;
    cv::cvtColor(projection, bgr_img, cv::COLOR_RGB2BGR);
    cv::imwrite(save_folder_ + "/projection_" + time_str + ".jpg", bgr_img);
  }
  STOP_TIMING_AVG

  return true;
}

bool ImageProjection::projectLatestImages(const PixelMapping& pixel_mapping, cv::UMat& projection, ros::Time& stamp, std::string& encoding) const
{
  auto images = getLatestImages(stamp, encoding);
  return projectImages(images, pixel_mapping, projection);
}

kalibr_image_geometry::CameraLoader& ImageProjection::getCameraLoader()
{
  return camera_loader_;
}

}

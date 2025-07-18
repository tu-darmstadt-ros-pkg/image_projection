#include <image_projection/periodic_image_projection.h>

#include <image_projection/utils/utils.h>

#include <functional>
#include <chrono>

using namespace std::chrono_literals;
namespace image_projection {

PeriodicImageProjection::PeriodicImageProjection(const rclcpp::Node::SharedPtr node)
    : node_(node),
      enabled_(false),
      publish_tf_(false),
      last_tf_stamp_(node->get_clock()->now()),
      image_projection_lib_(node),
      it_(node),
      always_recompute_mapping_(false)
{
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
}

bool PeriodicImageProjection::init()
{
  // Load parameters
  std::string projection_type;
  loadMandatoryParameter(node_, "projection_type", projection_type);
  loadMandatoryParameter(node_, "base_frame", base_frame_);
  node_->declare_parameter("always_recompute_mapping", false);
  node_->get_parameter("always_recompute_mapping", always_recompute_mapping_);
  node_->declare_parameter("encoding", "");
  node_->get_parameter("encoding", encoding_);

  /*
  node_->declare_parameter("pose", std::vector<double>(6, 0));
  std::vector<double> pose_vec = node_->get_parameter("pose").as_double_array();
  if (pose_vec.size() != 6) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Pose offset has to have a size of 6");
    pose_vec.resize(6, 0);
  }*/

  pose_param_sub_ = hector::createReconfigurableParameter(
      node_, "pose", std::ref(pose_vec_), "Pose of the virtual sensor in the base frame",
      hector::ParameterOptions<std::vector<double>>()
          .onValidate([](const auto& value) {
            if (value.size() != 6) {
              // RCLCPP_ERROR(rclcpp::get_logger("PeriodicImageProjection"), "Pose has invalid size.");
              return false;
            }
            return true;
          })
          .onUpdate(std::bind(&PeriodicImageProjection::poseParamCallback, this, std::placeholders::_1)));

  optical_frame_transform_ =
      Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX());
  updateSensorPose(pose_vec_[0], pose_vec_[1], pose_vec_[2], pose_vec_[3], pose_vec_[4], pose_vec_[5]);

  // Initialize projection
  projection_ = image_projection_lib_.loadProjectionPlugin(projection_type);
  if (!projection_) {
    return false;
  }

  std::string ns = "projection_parameters";
  projection_->initialize(node_, ns);
  projection_->loadParameters();
  initProjectionMat();

  std::stringstream topic_stream;
  topic_stream << node_->get_namespace();
  if (node_->get_namespace() != std::string("/")) {
    topic_stream << "/";
  }
  topic_stream << node_->get_name() << "/project_pixel_to_ray";
  pixel_to_ray_srv_ = node_->create_service<image_projection_msgs::srv::ProjectPixelTo3DRay>(
      topic_stream.str(),
      std::bind(&PeriodicImageProjection::projectPixelToRayCb, this, std::placeholders::_1, std::placeholders::_2));

  pose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>(
      "set_pose", 10, std::bind(&PeriodicImageProjection::poseCallback, this, std::placeholders::_1));

  // Publish virtual sensor frame tf
  node_->declare_parameter("publish_tf", true);
  node_->declare_parameter("virtual_sensor_frame", std::string(""));
  node_->declare_parameter("virtual_sensor_optical_frame", std::string(""));
  node_->get_parameter("publish_tf", publish_tf_);
  node_->get_parameter("virtual_sensor_frame", virtual_sensor_frame_);
  node_->get_parameter("virtual_sensor_optical_frame", virtual_sensor_optical_frame_);

  if (!virtual_sensor_optical_frame_.empty()) {
    optical_transform_msg_ = tf2::eigenToTransform(optical_frame_transform_);
    optical_transform_msg_.header.frame_id = virtual_sensor_frame_;
    optical_transform_msg_.child_frame_id = virtual_sensor_optical_frame_;
  }
  if (publish_tf_) {
    tf_timer_ = node_->create_wall_timer(50ms, std::bind(&PeriodicImageProjection::publishTfTimerCallback, this));
  }

  // Image publisher
  image_transport::SubscriberStatusCallback connect_cb = std::bind(&PeriodicImageProjection::connectCb, this);
  std::lock_guard<std::mutex> lock(connect_mutex_);
  // TODO reenable once image transport ports functionality
  image_pub_ = it_.advertise("projection", 1);  //, connect_cb, connect_cb);
  // TODO remove this workaround once image transport adds the callback
  connection_check_timer_ = node_->create_wall_timer(1s, std::bind(&PeriodicImageProjection::connectCb, this));
  return true;
}

void PeriodicImageProjection::initProjectionMat()
{
  pixel_mapping_.clear();
  projected_image_ =
      cv::UMat(projection_->imageHeight(), projection_->imageWidth(), CV_8UC3, cv::Scalar::all(0), cv::USAGE_DEFAULT);
}

// TODO this must currently be called periodically as image transport has not updated the correct function
void PeriodicImageProjection::connectCb()
{
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (image_pub_.getNumSubscribers() == 0) {
    RCLCPP_INFO_EXPRESSION(node_->get_logger(), enabled_, "No subscribers. Stopping image projection.");
    enabled_ = false;
    image_projection_lib_.getCameraLoader().stopImageSubscribers();
  } else {
    if (!enabled_) {
      RCLCPP_INFO(node_->get_logger(), "Have subscriber. Starting image projection.");
      image_projection_lib_.getCameraLoader().startImageSubscribers();
      enabled_ = true;
    }
  }
}

void PeriodicImageProjection::poseParamCallback(std::vector<double> pose_vec)
{
  std::scoped_lock<std::recursive_mutex> mutex_lock(reconfigure_mutex_);
  updateSensorPose(pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5]);
  initProjectionMat();
  publishCameraFrameToTf();
}

void PeriodicImageProjection::poseCallback(const std::shared_ptr<geometry_msgs::msg::Pose const> pose)
{
  std::scoped_lock<std::recursive_mutex> mutex_lock(reconfigure_mutex_);
  tf2::fromMsg(*pose, virtual_sensor_pose_);
  updateSensorPose(virtual_sensor_pose_);
  initProjectionMat();
  publishCameraFrameToTf();
}

void PeriodicImageProjection::projectAndPublishLatestImages()
{
  std::scoped_lock<std::recursive_mutex> mutex_lock(reconfigure_mutex_);
  // Check if projection has been initialized
  if (!projection_) {
    return;
  }

  // Do not compute the mapping yet, if we are disabled and we compute the mapping each time anyway
  if (!enabled_ && !always_recompute_mapping_) {
    return;
  }

  // Check if projection parameters have changed
  if (projection_->mappingChanged()) {
    initProjectionMat();
  }

  // Get images first
  rclcpp::Time stamp;
  auto images = image_projection_lib_.getLatestImages(stamp, encoding_);
  if (stamp == last_image_stamp_) {
    // No new images received
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
                         "No new images received. Skipping projection. This message is throttled.");
    return;
  }

  if (pixel_mapping_.empty() || always_recompute_mapping_) {
    // If no mapping is saved and camera info is available, compute it
    if (image_projection_lib_.getCameraLoader().cameraInfosReceived()) {
      pixel_mapping_ =
          image_projection_lib_.createMapping(projection_, base_frame_, stamp, virtual_sensor_optical_pose_);
      // Check if pixel mapping has been successful
      if (pixel_mapping_.empty()) {
        return;
      }
      projection_->unsetMappingChanged();
    } else {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3,
                                  "No camera info received yet. This message is throttled.");
      return;
    }
  }

  if (!enabled_) {
    return;
  }

  if (!image_projection_lib_.projectImages(images, pixel_mapping_, projected_image_)) {
    return;
  }

  // Convert to sensor msg
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = virtual_sensor_optical_frame_;
  cv_bridge::CvImage cv_image(header, encoding_, projected_image_.getMat(cv::ACCESS_READ));
  image_pub_.publish(cv_image.toImageMsg());
}

bool PeriodicImageProjection::projectPixelToRayCb(
    const image_projection_msgs::srv::ProjectPixelTo3DRay::Request::SharedPtr req,
    image_projection_msgs::srv::ProjectPixelTo3DRay::Response::SharedPtr resp)
{
  if (!projection_) {
    return false;
  }
  // TODO what to do if projection was changed?
  Eigen::Vector2d pixel(req->pixel.point.x, req->pixel.point.y);
  Eigen::Vector3d ray = projection_->targetImagePixelToProjectionSurfacePoint(pixel);
  ray = virtual_sensor_optical_pose_ * ray;
  ray = ray / ray.norm();
  resp->ray.point = tf2::toMsg(ray);
  resp->ray.header.frame_id = base_frame_;
  resp->ray.header.stamp = node_->get_clock()->now();
  return true;
}

void PeriodicImageProjection::publishTfTimerCallback()
{
  publishCameraFrameToTf();
}

void PeriodicImageProjection::publishCameraFrameToTf()
{
  if (virtual_sensor_frame_.empty()) {
    return;
  }
  rclcpp::Time now = node_->get_clock()->now();
  if (now == last_tf_stamp_) {
    return;
  }
  last_tf_stamp_ = now;
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg = tf2::eigenToTransform(virtual_sensor_pose_);
  transform_msg.header.stamp = now;
  transform_msg.header.frame_id = base_frame_;
  transform_msg.child_frame_id = virtual_sensor_frame_;
  tf_broadcaster_->sendTransform(transform_msg);

  if (virtual_sensor_optical_frame_.empty()) {
    return;
  }
  optical_transform_msg_.header.stamp = now;
  tf_broadcaster_->sendTransform(optical_transform_msg_);
}

void PeriodicImageProjection::updateSensorPose(const Eigen::Isometry3d& sensor_pose)
{
  virtual_sensor_pose_ = sensor_pose;
  virtual_sensor_optical_pose_ = sensor_pose * optical_frame_transform_;
}

void PeriodicImageProjection::updateSensorPose(double x, double y, double z, double roll, double pitch, double yaw)
{
  Eigen::Isometry3d sensor_pose(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  sensor_pose.translation() = Eigen::Vector3d(x, y, z);
  updateSensorPose(sensor_pose);
}

}  // namespace image_projection

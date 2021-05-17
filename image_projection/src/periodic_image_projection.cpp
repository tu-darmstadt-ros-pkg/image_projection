#include <image_projection/periodic_image_projection.h>

#include <image_projection/utils/utils.h>

namespace image_projection {

PeriodicImageProjection::PeriodicImageProjection(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), enabled_(false), publish_tf_(false), last_tf_stamp_(0), image_projection_lib_(nh, pnh), it_(pnh_), always_recompute_mapping_(false)
{}

bool PeriodicImageProjection::init()
{

  // Load parameters
  std::string projection_type;
  loadMandatoryParameter(pnh_, "projection_type", projection_type);
  loadMandatoryParameter(pnh_, "base_frame", base_frame_);
  pnh_.param("always_recompute_mapping", always_recompute_mapping_, false);
  pnh_.param<std::string>("encoding", encoding_, "");

  std::vector<double> pose_vec = pnh_.param("pose", std::vector<double>(6, 0));
  if (pose_vec.size() != 6) {
    ROS_ERROR_STREAM("Pose offset has to have a size of 6");
    pose_vec.resize(6, 0);
  }

  optical_frame_transform_ = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ())
                             * Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());
  updateSensorPose(pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], pose_vec[5]);

  // Initialize projection
  projection_ = image_projection_lib_.loadProjectionPlugin(projection_type);
  if (!projection_) {
    return false;
  }
  ros::NodeHandle projection_nh(pnh_, "projection_parameters");
  projection_->initialize(nh_, pnh_);
  projection_->loadParametersFromNamespace(projection_nh);
  initProjectionMat();

  pixel_to_ray_srv_ = pnh_.advertiseService("project_pixel_to_ray", &PeriodicImageProjection::projectPixelToRayCb, this);

  // Set up dynamic reconfigure
  reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<image_projection::ProjectionConfig>>(reconfigure_mutex_, pnh_);
  updateReconfigureConfig(pose_vec);
  reconfigure_server_->setCallback(boost::bind(&PeriodicImageProjection::dynamicReconfigureCallback, this, _1, _2));

  pose_sub_ = pnh_.subscribe<geometry_msgs::Pose>("set_pose", 10, &PeriodicImageProjection::poseCallback, this);

  // Publish virtual sensor frame tf
  pnh_.param("publish_tf", publish_tf_, true);
  pnh_.param("virtual_sensor_frame", virtual_sensor_frame_, std::string(""));
  pnh_.param("virtual_sensor_optical_frame", virtual_sensor_optical_frame_, std::string(""));
  if (!virtual_sensor_optical_frame_.empty()) {

    optical_transform_msg_.header.frame_id = virtual_sensor_frame_;
    optical_transform_msg_.child_frame_id = virtual_sensor_optical_frame_;
    tf::transformEigenToMsg(optical_frame_transform_, optical_transform_msg_.transform);
  }
  if (publish_tf_) {
    tf_timer_ =  nh_.createTimer(ros::Duration(0.05), &PeriodicImageProjection::publishTfTimerCallback, this, false);
  }

  // Image publisher
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&PeriodicImageProjection::connectCb, this);
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  image_pub_ = it_.advertise("projection", 1, connect_cb, connect_cb);
  return true;
}

void PeriodicImageProjection::initProjectionMat()
{
  pixel_mapping_.clear();
  projected_image_ = cv::UMat(projection_->imageHeight(), projection_->imageWidth(), CV_8UC3, cv::Scalar::all(0), cv::USAGE_DEFAULT);
}

void PeriodicImageProjection::connectCb() {
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (image_pub_.getNumSubscribers() == 0) {
    enabled_ = false;
    image_projection_lib_.getCameraLoader().stopImageSubscribers();
  } else {
    if (!enabled_) {
      image_projection_lib_.getCameraLoader().startImageSubscribers();
      enabled_ = true;
    }
  }
}

void PeriodicImageProjection::dynamicReconfigureCallback(ProjectionConfig& config, uint32_t /*level*/)
{
  boost::recursive_mutex::scoped_lock mutex_lock(reconfigure_mutex_);
  ROS_INFO("Reconfigure request: Pose: [%f, %f, %f, %f, %f, %f]",
           config.pose_x, config.pose_y, config.pose_z, config.pose_roll, config.pose_pitch, config.pose_yaw);
  updateSensorPose(config.pose_x, config.pose_y, config.pose_z, config.pose_roll, config.pose_pitch, config.pose_yaw);
  initProjectionMat();
  publishCameraFrameToTf();
}

void PeriodicImageProjection::poseCallback(const geometry_msgs::PoseConstPtr& pose)
{
  boost::recursive_mutex::scoped_lock mutex_lock(reconfigure_mutex_);
  tf::poseMsgToEigen(*pose, virtual_sensor_pose_);
  updateSensorPose(virtual_sensor_pose_);
  initProjectionMat();
  publishCameraFrameToTf();
}

void PeriodicImageProjection::updateReconfigureConfig(const std::vector<double>& pose_vec)
{
  if (pose_vec.size() != 6) {
    ROS_ERROR_STREAM("Pose has invalid size.");
    return;
  }
  image_projection::ProjectionConfig config;
  config.pose_x = pose_vec[0];
  config.pose_y = pose_vec[1];
  config.pose_z = pose_vec[2];
  config.pose_roll = pose_vec[3];
  config.pose_pitch = pose_vec[4];
  config.pose_yaw = pose_vec[5];

  boost::recursive_mutex::scoped_lock mutex_lock(reconfigure_mutex_);
  reconfigure_server_->updateConfig(config);
}

void PeriodicImageProjection::projectAndPublishLatestImages()
{
  boost::recursive_mutex::scoped_lock mutex_lock(reconfigure_mutex_);
  // Check if projection has been initialized
  if (!projection_) {
    return;
  }

  // Do not compute the mapping yet, if we are disabled and we compute the mapping each time anyway
  if (!enabled_ && always_recompute_mapping_) {
    return;
  }

  // Check if projection parameters have changed
  if (projection_->mappingChanged()) {
    initProjectionMat();
  }

  // Get images first
  ros::Time stamp;
  auto images = image_projection_lib_.getLatestImages(stamp, encoding_);

  if (pixel_mapping_.empty() || always_recompute_mapping_) {
    // If no mapping is saved and camera info is available, compute it
    if (image_projection_lib_.getCameraLoader().cameraInfosReceived()) {
      pixel_mapping_ = image_projection_lib_.createMapping(projection_, base_frame_, stamp, virtual_sensor_optical_pose_);
      // Check if pixel mapping has been successful
      if (pixel_mapping_.empty()) {
        return;
      }
      projection_->unsetMappingChanged();
    } else {
      ROS_WARN_STREAM_THROTTLE(1, "No camera info received yet.");
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
  std_msgs::Header header;
  header.stamp = stamp;
  header.frame_id = virtual_sensor_optical_frame_;
  cv_bridge::CvImage cv_image(header, encoding_, projected_image_.getMat(cv::ACCESS_READ));
  image_pub_.publish(cv_image.toImageMsg());
}

bool PeriodicImageProjection::projectPixelToRayCb(image_projection_msgs::ProjectPixelTo3DRay::Request& req, image_projection_msgs::ProjectPixelTo3DRay::Response &resp) {
  if (!projection_) {
    return false;
  }
  // TODO what to do if projection was changed?
  Eigen::Vector2d pixel(req.pixel.point.x, req.pixel.point.y);
  Eigen::Vector3d ray = projection_->targetImagePixelToProjectionSurfacePoint(pixel);
  ray = virtual_sensor_optical_pose_ * ray;
  ray = ray / ray.norm();
  tf::pointEigenToMsg(ray, resp.ray.point);
  resp.ray.header.frame_id = base_frame_;
  resp.ray.header.stamp = ros::Time::now();
  return true;
}

void PeriodicImageProjection::publishTfTimerCallback(const ros::TimerEvent& /*event*/)
{
  publishCameraFrameToTf();
}

void PeriodicImageProjection::publishCameraFrameToTf()
{
  if (virtual_sensor_frame_.empty()) {
    return;
  }
  ros::Time now = ros::Time::now();
  if (now == last_tf_stamp_) {
    return;
  }
  last_tf_stamp_ = now;
  geometry_msgs::TransformStamped transform_msg;
  transform_msg.header.stamp = now;
  transform_msg.header.frame_id = base_frame_;
  transform_msg.child_frame_id = virtual_sensor_frame_;
  tf::transformEigenToMsg(virtual_sensor_pose_, transform_msg.transform);
  tf_broadcaster_.sendTransform(transform_msg);

  if (virtual_sensor_optical_frame_.empty()) {
    return;
  }
  optical_transform_msg_.header.stamp = now;
  tf_broadcaster_.sendTransform(optical_transform_msg_);
}

void PeriodicImageProjection::updateSensorPose(const Eigen::Isometry3d& sensor_pose) {
  virtual_sensor_pose_ = sensor_pose;
  virtual_sensor_optical_pose_ = sensor_pose * optical_frame_transform_;
}

void PeriodicImageProjection::updateSensorPose(double x, double y, double z, double roll, double pitch, double yaw) {
  Eigen::Isometry3d sensor_pose(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                                           * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                                           * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  sensor_pose.translation() = Eigen::Vector3d(x, y, z);
  updateSensorPose(sensor_pose);
}

}

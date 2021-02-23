#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "periodic_image_projection_node");
  nodelet::Loader nodelet;
  const nodelet::M_string& remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  const std::string& nodelet_name = ros::this_node::getName();
  ROS_INFO_STREAM("Started " << nodelet_name << " nodelet.");
  nodelet.load(nodelet_name, "image_projection/PeriodicImageProjectionNodelet", remap, nargv);
  ros::spin();
  return 0;
}

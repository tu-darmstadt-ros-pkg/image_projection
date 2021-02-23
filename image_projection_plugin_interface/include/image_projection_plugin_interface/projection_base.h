#ifndef IMAGE_PROJECTION_PLUGIN_INTERFACE_PROJECTIONS_PROJECTION_BASE_H
#define IMAGE_PROJECTION_PLUGIN_INTERFACE_PROJECTIONS_PROJECTION_BASE_H

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace image_projection_plugin_interface {

class ProjectionBase {
public:
  virtual ~ProjectionBase();
  virtual bool initialize(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  bool loadParametersFromNamespace(const ros::NodeHandle& nh);

  bool mappingChanged() const;
  void unsetMappingChanged();

  virtual Eigen::Vector2d projectionSurfacePointToTargetImagePixel(const Eigen::Vector3d& point) const = 0;
  virtual Eigen::Vector3d targetImagePixelToProjectionSurfacePoint(const Eigen::Vector2d& target_image_pixel) const = 0;

  void setImageWidth(int image_width);
  int imageWidth() const;
  void setImageHeight(int image_height);
  int imageHeight() const;


protected:
  virtual bool loadProjectionParametersFromNamespace(const ros::NodeHandle& nh) = 0;
  void registerParameter(const std::string& name, double value, const std::string& description, double min, double max);
  void registerParameterFromNamespace(const ros::NodeHandle& nh, const std::string& name, double default_value, const std::string& description, double min, double max);
  double getParameter(const std::string& name) const;
  virtual void parametersChanged();
private:
  bool loadBaseParametersFromNamespace(const ros::NodeHandle& nh);
  void reconfigureCallback(const std::string& name, double value);
  void imageDimensionsReconfigureCallback(int* field, int value);
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> reconfigure_;
  std::map<std::string, double> config_; /// Configuration parameters of the projection
  bool mapping_changed_;

  // Parameters
  int image_width_;
  int image_height_;
  bool frame_id_; /// Name of frame this projection is defined in

};

}

#endif

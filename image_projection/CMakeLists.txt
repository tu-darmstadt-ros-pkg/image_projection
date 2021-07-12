cmake_minimum_required(VERSION 3.0.2)
project(image_projection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  kalibr_camera_loader
  nodelet
  roscpp
  sensor_msgs
  tf2_ros
  eigen_conversions
  image_transport
  image_projection_msgs
  image_projection_plugin_interface
  dynamic_reconfigure
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
   cfg/Projection.cfg
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES image_projection
  CATKIN_DEPENDS
    kalibr_camera_loader
    nodelet
    roscpp
    sensor_msgs
    tf2_ros
    eigen_conversions
    image_projection_msgs
    image_projection_plugin_interface
    dynamic_reconfigure
    image_transport
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

set(HEADERS
  include/${PROJECT_NAME}/image_projection.h
  include/${PROJECT_NAME}/utils/timing.h
  include/${PROJECT_NAME}/utils/utils.h
  include/${PROJECT_NAME}/periodic_image_projection.h
  include/${PROJECT_NAME}/image_projection_server.h
)

set(SOURCES
  src/image_projection.cpp
  src/periodic_image_projection.cpp
  src/image_projection_server.cpp
  src/periodic_image_projection_nodelet.cpp
  src/image_projection_server_nodelet.cpp
  src/utils/utils.cpp
)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(periodic_image_projection_node src/periodic_image_projection_node.cpp)
target_link_libraries(periodic_image_projection_node ${PROJECT_NAME})

add_executable(image_projection_server_node src/image_projection_server_node.cpp)
target_link_libraries(image_projection_server_node ${PROJECT_NAME})

install(DIRECTORY 
  config
  cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} periodic_image_projection_node image_projection_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES 
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

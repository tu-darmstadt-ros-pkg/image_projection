cmake_minimum_required(VERSION 3.0.2)
project(image_projection_demo)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package()

#############
## Install ##
#############

install(DIRECTORY 
  launch
  config
  rqt
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

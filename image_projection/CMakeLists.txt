cmake_minimum_required(VERSION 3.8)
project(image_projection)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()
#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
set(DEPENDENCIES
  rclcpp
  rclcpp_components
  hector_ros2_utils
  extended_camera_loader
  sensor_msgs
  tf2_ros
  tf2_eigen
  image_transport
  image_projection_msgs
  image_projection_plugin_interface
  OpenCV
  pluginlib
  #dynamic_reconfigure
)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
    find_package(${Dependency} REQUIRED)
endforeach()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
## Generate dynamic reconfigure parameters in the 'cfg' folder

#generate_dynamic_reconfigure_options(
#  cfg/Projection.cfg
#)

###################################
## catkin specific configuration ##
###################################
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_projection
#  CATKIN_DEPENDS
#    kalibr_camera_loader
#    nodelet
#    roscpp
#    sensor_msgs
#    tf2_ros
#    eigen_conversions
#    image_projection_msgs
#    image_projection_plugin_interface
#    dynamic_reconfigure
#    image_transport
#  DEPENDS system_lib
#)

###########
## Build ##
###########

#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
#)

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
  src/periodic_image_projection_node.cpp
  #src/periodic_image_projection_nodelet.cpp
  #src/image_projection_server_nodelet.cpp
  src/utils/utils.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADERS} ${SOURCES})
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
#target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
#add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

add_library(periodic_image_projection src/periodic_image_projection.cpp)
ament_target_dependencies(periodic_image_projection
  rclcpp
  rclcpp_components
)
target_link_libraries(periodic_image_projection ${PROJECT_NAME})


rclcpp_components_register_node(periodic_image_projection
  PLUGIN "image_projection::PeriodicImageProjection"
  EXECUTABLE periodic_image_projection_node
)

add_library(image_projection_server src/image_projection_server.cpp)
ament_target_dependencies(image_projection_server
  rclcpp
  rclcpp_components
)
target_link_libraries(image_projection_server ${PROJECT_NAME})


rclcpp_components_register_node(image_projection_server
  PLUGIN "image_projection::ImageProjectionServer"
  EXECUTABLE image_projection_server_node
)

add_executable(periodic_image_projection_standalone_node src/periodic_image_projection_standalone_node.cpp)
target_link_libraries(periodic_image_projection_standalone_node ${PROJECT_NAME})

add_executable(image_projection_server_standalone_node src/image_projection_server_standalone_node.cpp)
target_link_libraries(image_projection_server_standalone_node ${PROJECT_NAME})

# Install
ament_export_targets(export_image_projection HAS_LIBRARY_TARGET)
install(TARGETS ${PROJECT_NAME}
  periodic_image_projection
  image_projection_server
  periodic_image_projection_standalone_node
  image_projection_server_standalone_node
  EXPORT export_${PROJECT_NAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)


install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)

# install(TARGETS ${PROJECT_NAME}  
#   ARCHIVE DESTINATION lib/${PROJECT_NAME}
#   LIBRARY DESTINATION lib/${PROJECT_NAME}
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

ament_package()

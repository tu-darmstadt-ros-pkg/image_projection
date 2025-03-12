cmake_minimum_required(VERSION 3.8)
project(image_projection_plugin_interface)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hector_ros2_utils REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(include)

add_library(${PROJECT_NAME} SHARED
  include/${PROJECT_NAME}/projection_base.h
  src/projection_base.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp hector_ros2_utils)

# Install library and include folder
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_package()

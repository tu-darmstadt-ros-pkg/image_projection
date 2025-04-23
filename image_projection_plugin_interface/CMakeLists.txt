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
find_package(Eigen3 REQUIRED)
set(DEPENDENCIES
        rclcpp
        hector_ros2_utils
)

foreach(Dependency IN ITEMS ${DEPENDENCIES})
    find_package(${Dependency} REQUIRED)
endforeach()

# Library
add_library(${PROJECT_NAME} SHARED
        include/${PROJECT_NAME}/projection_base.h
        src/projection_base.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

# Install
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES} Eigen3)
ament_package()

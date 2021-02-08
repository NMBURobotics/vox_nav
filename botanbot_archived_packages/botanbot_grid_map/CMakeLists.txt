cmake_minimum_required(VERSION 3.5)
project(archived_botanbot_grid_map)

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_filters REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_octomap REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_visualization REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(botanbot_utilities REQUIRED)
find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

grid_map_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  # ${OCTOMAP_INCLUDE_DIR}
)

set(dependencies
  rclcpp
  grid_map_ros
  grid_map_msgs
  grid_map_pcl
  botanbot_utilities
  OpenCV
)

set(grid_map_executable_name archived_botanbot_grid_map_node)

add_executable(${grid_map_executable_name}
  src/archived_botanbot_grid_map.cpp
)

ament_target_dependencies(${grid_map_executable_name}
  ${dependencies}
)

target_link_libraries(${grid_map_executable_name} )

 
install(TARGETS ${grid_map_executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY config launch rviz 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
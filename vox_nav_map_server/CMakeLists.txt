cmake_minimum_required(VERSION 3.5)
project(vox_nav_map_server CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vox_nav_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(robot_localization REQUIRED)
find_package(vox_nav_utilities REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OCTOMAP REQUIRED)

set(dependencies
rclcpp
rclcpp_action
visualization_msgs
geometry_msgs
sensor_msgs
octomap_msgs
tf2_geometry_msgs
vox_nav_msgs
tf2_eigen
tf2_ros
pcl_ros
robot_localization
vox_nav_utilities
Eigen3
OCTOMAP
)

include_directories(
  include
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS})

set(vox_nav_map_manager_exc_name vox_nav_map_manager)
add_executable(${vox_nav_map_manager_exc_name} src/map_manager.cpp)
ament_target_dependencies(${vox_nav_map_manager_exc_name} ${dependencies})
target_link_libraries(${vox_nav_map_manager_exc_name})
 
install(TARGETS ${vox_nav_map_manager_exc_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(
  DIRECTORY 
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
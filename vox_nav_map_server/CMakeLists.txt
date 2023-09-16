cmake_minimum_required(VERSION 3.5)
project(vox_nav_map_server C CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vox_nav_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(vox_nav_utilities REQUIRED)
find_package(vox_nav_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(rviz_default_plugins REQUIRED)

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
rviz_default_plugins
)

include_directories(include)

add_executable(map_manager src/map_manager.cpp)
ament_target_dependencies(map_manager ${dependencies})

add_executable(osm_map_manager src/osm_map_manager.cpp)
ament_target_dependencies(osm_map_manager ${dependencies}) 

add_executable(map_manager_no_gps src/map_manager_no_gps.cpp)
ament_target_dependencies(map_manager_no_gps ${dependencies})

install(TARGETS map_manager
                map_manager_no_gps
                osm_map_manager  
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/)

install(
  DIRECTORY config launch urdf meshes
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
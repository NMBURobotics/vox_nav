cmake_minimum_required(VERSION 3.5)
project(botanbot_map_server)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(botanbot_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(robot_localization REQUIRED)
find_package(botanbot_utilities REQUIRED)
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
botanbot_msgs
tf2_eigen
tf2_ros
pcl_ros
robot_localization
botanbot_utilities
Eigen3
OCTOMAP
)

include_directories(
  include
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS})

set(botanbot_map_server_exc_name botanbot_map_server)
add_executable(${botanbot_map_server_exc_name} src/botanbot_map_server.cpp)
ament_target_dependencies(${botanbot_map_server_exc_name} ${dependencies})
target_link_libraries(${botanbot_map_server_exc_name})

 
install(TARGETS ${botanbot_map_server_exc_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(
  DIRECTORY launch params 
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
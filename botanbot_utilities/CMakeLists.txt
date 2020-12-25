cmake_minimum_required(VERSION 3.5)
project(botanbot_utilities)

find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(nav2_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(Eigen3 REQUIRED)
set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
 

nav2_package()

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav2_util
  pcl_ros
  Eigen3
)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
)

add_library(tf_helpers SHARED
  src/tf_helpers.cpp
  src/pcl_helpers.cpp
)

ament_target_dependencies(tf_helpers ${dependencies})

add_executable(gps_waypoint_collector src/gps_waypoint_collector.cpp)
target_link_libraries(gps_waypoint_collector)
ament_target_dependencies(gps_waypoint_collector ${dependencies})

install(TARGETS tf_helpers
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(TARGETS gps_waypoint_collector
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/
)

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR})
ament_export_libraries(tf_helpers)
ament_export_dependencies(${dependencies})

ament_package()

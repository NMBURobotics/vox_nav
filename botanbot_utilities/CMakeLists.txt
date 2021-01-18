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
find_package(PCL REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

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
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS})

add_library(tf_helpers SHARED
  src/tf_helpers.cpp
  src/pcl_helpers.cpp
)
ament_target_dependencies(tf_helpers ${dependencies})

add_library(navsat_conversions SHARED
  src/navsat_conversions.cpp
)
ament_target_dependencies(navsat_conversions ${dependencies})

add_library(gps_waypoint_collector SHARED
  src/gps_waypoint_collector.cpp
)
ament_target_dependencies(gps_waypoint_collector ${dependencies})

add_executable(gps_waypoint_collector_node src/gps_waypoint_collector_node.cpp)
target_link_libraries(gps_waypoint_collector_node gps_waypoint_collector)
ament_target_dependencies(gps_waypoint_collector_node ${dependencies})

add_executable(pcl2octomap_converter  src/pcl_helpers.cpp
                                      src/pcl2octomap_converter.cpp)
target_link_libraries(pcl2octomap_converter ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
ament_target_dependencies(pcl2octomap_converter ${dependencies})

install(TARGETS tf_helpers navsat_conversions gps_waypoint_collector
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS gps_waypoint_collector_node pcl2octomap_converter
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/)

install(DIRECTORY config launch 
        DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR})
ament_export_libraries(tf_helpers navsat_conversions gps_waypoint_collector)
ament_export_dependencies(${dependencies})

ament_package()

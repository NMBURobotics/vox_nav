cmake_minimum_required(VERSION 3.5)
project(botanbot_utilities)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED "fcl>=0.5.0")
# replace LIBFCL_LIBRARIES with full path to the library
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(octomap_msgs REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
    include
    ${LIBFCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${THIS_PACKAGE_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS})

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  Eigen3
  OCTOMAP
  octomap_msgs
  ompl
)

add_library(tf_helpers SHARED src/tf_helpers.cpp
                              src/pcl_helpers.cpp)
ament_target_dependencies(tf_helpers ${dependencies})
add_library(gps_waypoint_collector SHARED src/gps_waypoint_collector.cpp)
ament_target_dependencies(gps_waypoint_collector ${dependencies})

add_executable(gps_waypoint_collector_node src/gps_waypoint_collector_node.cpp)
target_link_libraries(gps_waypoint_collector_node gps_waypoint_collector)
ament_target_dependencies(gps_waypoint_collector_node ${dependencies})

add_executable(pcl2octomap_converter  src/pcl_helpers.cpp
                                      src/pcl2octomap_converter.cpp)
target_link_libraries(pcl2octomap_converter ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
ament_target_dependencies(pcl2octomap_converter ${dependencies})

set(planner_benchmarking_exc_name planner_benchmarking)
add_executable(${planner_benchmarking_exc_name} src/planner_benchmarking.cpp)
ament_target_dependencies(${planner_benchmarking_exc_name} ${dependencies})
target_link_libraries(${planner_benchmarking_exc_name} ${LIBFCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES})

install(TARGETS tf_helpers gps_waypoint_collector 
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS gps_waypoint_collector_node 
                pcl2octomap_converter 
                ${planner_benchmarking_exc_name} 
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/)

install(DIRECTORY config launch 
        DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR})
ament_export_libraries(tf_helpers gps_waypoint_collector)
ament_export_dependencies(${dependencies})

ament_package()

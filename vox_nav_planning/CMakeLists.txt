cmake_minimum_required(VERSION 3.5)
project(vox_nav_planning)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED "fcl>=0.7.0")
# replace LIBFCL_LIBRARIES with full paths to the libraries
set(LIBFCL_LIBRARIES_FULL "")
foreach(LIBFCL_LIBRARY ${LIBFCL_LIBRARIES})
  find_library(${LIBFCL_LIBRARY}_LIB ${LIBFCL_LIBRARY} ${LIBFCL_LIBRARY_DIRS})
  list(APPEND LIBFCL_LIBRARIES_FULL ${${LIBFCL_LIBRARY}_LIB})
endforeach()
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

# Try for OpenCV 4.X, but settle for whatever is installed
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif ()
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(vox_nav_msgs REQUIRED)
find_package(vox_nav_utilities REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include
                   ${OMPL_INCLUDE_DIRS})
                   
set(dependencies
rclcpp
pluginlib
geometry_msgs
octomap_msgs
sensor_msgs
vox_nav_msgs
vox_nav_utilities
std_msgs
tf2_eigen
tf2_geometry_msgs
tf2_ros
trajectory_msgs
visualization_msgs
OCTOMAP
LIBFCL
pcl_ros
OpenCV)

# Native Voxnav sampling-based PLANNERS ######################
add_library(vox_nav_ompl_planners SHARED  src/native_planners/RRTStarF.cpp
                                          src/native_planners/LQRPlanner.cpp
                                          src/native_planners/LQRRRTStar.cpp 
                                          src/native_planners/InformedSGCP.cpp
                                          src/native_planners/CostTrustKinoPlanner.cpp)
target_link_libraries(vox_nav_ompl_planners ${PCL_LIBRARIES})
ament_target_dependencies(vox_nav_ompl_planners ${dependencies})

# PLANNER SERVER ##############################################
add_executable(planner_server src/planner_server.cpp)
ament_target_dependencies(planner_server ${dependencies})
target_link_libraries(planner_server ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl)

# SE2 PLANNE R##################################################
set(se2_planner vox_nav_se2_planner)
add_library(se2_planner SHARED src/plugins/se2_planner.cpp)
ament_target_dependencies(se2_planner ${dependencies})
target_link_libraries(se2_planner ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl)

# SE3 PLANNE R##################################################
set(se3_planner vox_nav_se3_planner)
add_library(se3_planner SHARED src/plugins/se3_planner.cpp)
ament_target_dependencies(se3_planner ${dependencies})
target_link_libraries(se3_planner ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl)

# ELEVATION PLANNER ############################################
add_library(elevation_planner SHARED src/plugins/elevation_planner.cpp)
ament_target_dependencies(elevation_planner ${dependencies})
target_link_libraries(elevation_planner ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl)

# ELEVATION CONTROL PLANNER ############################################
add_library(elevation_control_planner SHARED src/plugins/elevation_control_planner.cpp)
ament_target_dependencies(elevation_control_planner ${dependencies})
target_link_libraries(elevation_control_planner ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl vox_nav_ompl_planners)

# OPTIMAL ELEVATION PLANNER ####################################
add_library(optimal_elevation_planner SHARED src/plugins/optimal_elevation_planner.cpp)
ament_target_dependencies(optimal_elevation_planner ${dependencies})
target_link_libraries(optimal_elevation_planner ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl)

# OSM ELEVATION PLANNER ####################################
add_library(osm_elevation_planner SHARED src/plugins/osm_elevation_planner.cpp)
ament_target_dependencies(osm_elevation_planner ${dependencies})
target_link_libraries(osm_elevation_planner ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl vox_nav_ompl_planners)

# POLYTUNNEL PLANNER ####################################
add_library(polytunnel_planner SHARED src/plugins/polytunnel_planner.cpp)
ament_target_dependencies(polytunnel_planner ${dependencies})
target_link_libraries(polytunnel_planner ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ${PCL_LIBRARIES} ompl)

# ELEVATION CONTROL PLANNERS BENCHMARKS ####################################
add_executable(car_control_planners_benchmark src/tools/car_control_planners_benchmark.cpp)
ament_target_dependencies(car_control_planners_benchmark ${dependencies})
target_link_libraries(car_control_planners_benchmark ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} vox_nav_ompl_planners ompl)

# QUADROTOR CONTROL PLANNERS BENCHMARKS ####################################
add_executable(quadrotor_control_planners_benchmark src/tools/quadrotor_control_planners_benchmark.cpp)
ament_target_dependencies(quadrotor_control_planners_benchmark ${dependencies})
target_link_libraries(quadrotor_control_planners_benchmark ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} vox_nav_ompl_planners ompl)

install(TARGETS optimal_elevation_planner 
                osm_elevation_planner
                elevation_planner
                elevation_control_planner
                polytunnel_planner
                se2_planner
                se3_planner
                vox_nav_ompl_planners
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS planner_server
                car_control_planners_benchmark
                quadrotor_control_planners_benchmark
                
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

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME})
        
ament_export_include_directories(include)
ament_export_libraries( optimal_elevation_planner
                        osm_elevation_planner 
                        elevation_planner
                        elevation_control_planner
                        polytunnel_planner
                        se2_planner
                        se3_planner)
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
ament_package()
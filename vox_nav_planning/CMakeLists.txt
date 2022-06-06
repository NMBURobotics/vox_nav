cmake_minimum_required(VERSION 3.5)
project(vox_nav_planning)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED "fcl>=0.5.0")
# replace LIBFCL_LIBRARIES with full path to the library
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

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
pcl_ros)

# RRTStarF  PLANNER ####################################
add_library(RRTStarF SHARED src/rrt/RRTStarF.cpp
                            src/rrt/LQRPlanner.cpp )
target_link_libraries(RRTStarF ${PCL_LIBRARIES})
ament_target_dependencies(RRTStarF ${dependencies})

# PLANNER SERVER ##############################################
add_executable(planner_server src/planner_server.cpp)
ament_target_dependencies(planner_server ${dependencies})
target_link_libraries(planner_server ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl)

# SE2 PLANNE R##################################################
set(se2_planner vox_nav_se2_planner)
add_library(se2_planner SHARED src/plugins/se2_planner.cpp)
ament_target_dependencies(se2_planner ${dependencies})
target_link_libraries(se2_planner ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl)

# ELEVATION PLANNER ############################################
add_library(elevation_planner SHARED src/plugins/elevation_planner.cpp)
ament_target_dependencies(elevation_planner ${dependencies})
target_link_libraries(elevation_planner ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl)

# ELEVATION CONTROL PLANNER ############################################
add_library(elevation_control_planner SHARED src/plugins/elevation_control_planner.cpp)
ament_target_dependencies(elevation_control_planner ${dependencies})
target_link_libraries(elevation_control_planner ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl RRTStarF)

# OPTIMAL ELEVATION PLANNER ####################################
add_library(optimal_elevation_planner SHARED src/plugins/optimal_elevation_planner.cpp)
ament_target_dependencies(optimal_elevation_planner ${dependencies})
target_link_libraries(optimal_elevation_planner ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl)

install(TARGETS optimal_elevation_planner 
                elevation_planner
                elevation_control_planner
                se2_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS planner_server
                RRTStarF
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
ament_export_libraries(optimal_elevation_planner
                       elevation_planner
                       se2_planner)
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
ament_package()
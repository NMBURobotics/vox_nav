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

set(vox_nav_planner_server_exc_name vox_nav_planner_server)
add_executable(${vox_nav_planner_server_exc_name} src/planner_server.cpp)
ament_target_dependencies(${vox_nav_planner_server_exc_name} ${dependencies})
target_link_libraries(${vox_nav_planner_server_exc_name} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl)

set(vox_nav_se3_planner_exc_name vox_nav_se3_planner)
add_library(${vox_nav_se3_planner_exc_name} SHARED src/plugins/se3_planner.cpp
                                                   src/plugins/se3_planner_utils.cpp)
ament_target_dependencies(${vox_nav_se3_planner_exc_name} ${dependencies})
target_link_libraries(${vox_nav_se3_planner_exc_name} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl)

set(vox_nav_elevation_planner_exc_name vox_nav_composite_planner)
add_library(${vox_nav_elevation_planner_exc_name} SHARED src/plugins/elevation_planner.cpp
                                                   src/plugins/se3_planner_utils.cpp
                                                   src/plugins/elevation_state_space.cpp)
ament_target_dependencies(${vox_nav_elevation_planner_exc_name} ${dependencies})
target_link_libraries(${vox_nav_elevation_planner_exc_name} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl)

set(vox_nav_dubins_3d_planner_exc_name vox_nav_dubins_3d_planner)
add_library(${vox_nav_dubins_3d_planner_exc_name} SHARED src/plugins/dubins_3d_planner.cpp
                                                   src/plugins/se3_planner_utils.cpp
                                                   src/plugins/dubins_3d_state_space.cpp)
ament_target_dependencies(${vox_nav_dubins_3d_planner_exc_name} ${dependencies})
target_link_libraries(${vox_nav_dubins_3d_planner_exc_name} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl)

set(vox_nav_se2_planner_exc_name vox_nav_se2_planner)
add_library(${vox_nav_se2_planner_exc_name} SHARED src/plugins/se2_planner.cpp)
ament_target_dependencies(${vox_nav_se2_planner_exc_name} ${dependencies})
target_link_libraries(${vox_nav_se2_planner_exc_name} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ompl)

install(TARGETS ${vox_nav_se3_planner_exc_name} 
                ${vox_nav_elevation_planner_exc_name} 
                ${vox_nav_dubins_3d_planner_exc_name}
                ${vox_nav_se2_planner_exc_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${vox_nav_planner_server_exc_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY  
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${vox_nav_se3_planner_exc_name} 
                       ${vox_nav_elevation_planner_exc_name}
                       ${vox_nav_dubins_3d_planner_exc_name}
                       ${vox_nav_se2_planner_exc_name})
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

ament_package()
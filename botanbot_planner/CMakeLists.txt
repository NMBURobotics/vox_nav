cmake_minimum_required(VERSION 3.5)
project(botanbot_planner)

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
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
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
find_package(botanbot_msgs REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(
  include
  ${LIBFCL_INCLUDE_DIRS}
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS})

set(dependencies
rclcpp
geometry_msgs
octomap_msgs
sensor_msgs
botanbot_msgs
std_msgs
tf2_eigen
tf2_geometry_msgs
tf2_ros
trajectory_msgs
visualization_msgs
OCTOMAP
LIBFCL
ompl
pcl_ros)

set(botanbot_se3_planner_exc_name botanbot_se3_planner)
add_executable(${botanbot_se3_planner_exc_name} src/botanbot_se3_planning.cpp)
ament_target_dependencies(${botanbot_se3_planner_exc_name} ${dependencies})
target_link_libraries(${botanbot_se3_planner_exc_name} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ${OMPL_LIBRARIES})

set(botanbot_se2_planner_exc_name botanbot_se2_planner)
add_executable(${botanbot_se2_planner_exc_name} src/botanbot_se2_planning.cpp)
ament_target_dependencies(${botanbot_se2_planner_exc_name} ${dependencies})
target_link_libraries(${botanbot_se2_planner_exc_name} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ${OMPL_LIBRARIES})

set(botanbot_se2_planner_control_space_exc_name botanbot_se2_planner_control_space)
add_executable(${botanbot_se2_planner_control_space_exc_name} src/botanbot_se2_control_space_planner.cpp)
ament_target_dependencies(${botanbot_se2_planner_control_space_exc_name} ${dependencies})
target_link_libraries(${botanbot_se2_planner_control_space_exc_name} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ${OMPL_LIBRARIES})

install(TARGETS ${botanbot_se3_planner_exc_name} ${botanbot_se2_planner_exc_name} ${botanbot_se2_planner_control_space_exc_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
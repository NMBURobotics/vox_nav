cmake_minimum_required(VERSION 3.5)
project(botanbot_control)

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
find_package(pluginlib REQUIRED)
find_package(ACADO REQUIRED)
find_package(botanbot_msgs REQUIRED)
find_package(botanbot_utilities REQUIRED)

include_directories(
  include
  ${LIBFCL_INCLUDE_DIRS}
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${ACADO_INCLUDE_DIRS}
  include/botanbot_control/mpc_controller/mpc_acado/
  ${ACADO_INCLUDE_DIRS}/../../share/acado/external_packages/qpoases/
  ${ACADO_INCLUDE_DIRS}/../../share/acado/external_packages/qpoases/INCLUDE/
  ${ACADO_INCLUDE_DIRS}/../../share/acado/external_packages/qpoases/SRC/
)

set(dependencies
    rclcpp
    pluginlib
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
    pcl_ros
    botanbot_msgs
    botanbot_utilities)

file(GLOB_RECURSE SOURCES ${ACADO_INCLUDE_DIRS}/../../share/acado/external_packages/qpoases/SRC/*.cpp 
                          include/botanbot_control/mpc_controller/mpc_acado/*.c
                          include/botanbot_control/mpc_controller/mpc_acado/*.cpp)
                          
add_library(mpc_lib SHARED ${SOURCES})
ament_target_dependencies(mpc_lib ${dependencies})
target_link_libraries(mpc_lib ${ACADO_SHARED_LIBRARIES})

set(mpc_controller_exc_name botanbot_planner_server)
add_executable(${mpc_controller_exc_name} src/mpc_controller/acado.cpp
                                          src/mpc_controller/mpc_controller.cpp)
ament_target_dependencies(${mpc_controller_exc_name} ${dependencies})
target_link_libraries(${mpc_controller_exc_name} ${OCTOMAP_LIBRARIES} ${LIBFCL_LIBRARIES} ${OMPL_LIBRARIES} mpc_lib)

install(TARGETS mpc_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${mpc_controller_exc_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
ament_export_libraries(mpc_lib)
ament_package()
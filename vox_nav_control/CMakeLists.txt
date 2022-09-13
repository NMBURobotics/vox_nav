cmake_minimum_required(VERSION 3.5)
project(vox_nav_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add this repository's cmake modules to CMAKE_MODULE_PATH
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

find_package(PkgConfig REQUIRED)
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
find_package(pcl_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(vox_nav_msgs REQUIRED)
find_package(vox_nav_utilities REQUIRED)
find_package(casadi REQUIRED)
find_package(ACADO REQUIRED)

include_directories(
  include
  include/vox_nav_control/mpc_controller_acado/auto_gen
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${casadi_INCLUDE_DIR}
  ${ACADO_INCLUDE_DIRS}
  ${ACADO_QPOASES_EMBEDDED_INC_DIRS}
 )

set(dependencies
    rclcpp
    pluginlib
    geometry_msgs
    sensor_msgs
    nav_msgs
    std_msgs
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    trajectory_msgs
    visualization_msgs
    pcl_ros
    vox_nav_msgs
    vox_nav_utilities)

add_executable(vox_nav_controller_server src/controller_server.cpp)
ament_target_dependencies(vox_nav_controller_server ${dependencies})
target_link_libraries(vox_nav_controller_server ompl mosquittopp mosquitto)

add_executable(mpc_controller_acado_code_gen src/mpc_controller_acado/mpc_controller_acado_code_gen.cpp)
ament_target_dependencies(mpc_controller_acado_code_gen ${dependencies})
target_link_libraries(mpc_controller_acado_code_gen ${ACADO_SHARED_LIBRARIES})

add_library(mpc_controller_casadi SHARED               src/mpc_controller_casadi/mpc_controller_casadi_core.cpp 
                                                       src/mpc_controller_casadi/mpc_controller_casadi_ros.cpp)
ament_target_dependencies(mpc_controller_casadi ${dependencies})
target_link_libraries(mpc_controller_casadi ompl casadi)

add_library(lyapunov_controller SHARED                 src/lyapunov_controller/lyapunov_controller_ros.cpp)
ament_target_dependencies(lyapunov_controller ${dependencies})
target_link_libraries(lyapunov_controller ompl casadi)

option(ACADO_CODE_IS_READY "Whether Acado code has been generated" ON)

if(ACADO_CODE_IS_READY)

  add_library(mpc_solver_acado SHARED   
              ${ACADO_QPOASES_EMBEDDED_SOURCES}
              include/vox_nav_control/mpc_controller_acado/auto_gen/acado_common.h
              include/vox_nav_control/mpc_controller_acado/auto_gen/acado_solver.c
              include/vox_nav_control/mpc_controller_acado/auto_gen/acado_integrator.c
              include/vox_nav_control/mpc_controller_acado/auto_gen/acado_qpoases_interface.hpp
              include/vox_nav_control/mpc_controller_acado/auto_gen/acado_qpoases_interface.cpp
              include/vox_nav_control/mpc_controller_acado/auto_gen/acado_auxiliary_functions.h
              include/vox_nav_control/mpc_controller_acado/auto_gen/acado_auxiliary_functions.c)

  add_library(mpc_controller_acado SHARED  src/mpc_controller_acado/mpc_controller_acado_ros.cpp)
  ament_target_dependencies(mpc_controller_acado ${dependencies})
  target_link_libraries(mpc_controller_acado ompl ${ACADO_SHARED_LIBRARIES} mpc_solver_acado)

  install(TARGETS mpc_controller_acado
                  mpc_solver_acado
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

  ament_export_libraries(mpc_controller_acado)
  ament_export_libraries(mpc_solver_acado)

endif(ACADO_CODE_IS_READY)

install(TARGETS mpc_controller_casadi
                lyapunov_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS vox_nav_controller_server
                mpc_controller_acado_code_gen
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY cmake launch 
DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(mpc_controller_casadi)
ament_export_libraries(lyapunov_controller)
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
ament_package()
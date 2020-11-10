################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(botanbot_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

option(ENABLE_PROFILER "Enable Ignition Profiler" TRUE)
if(ENABLE_PROFILER)
  find_package(ignition-common3 QUIET COMPONENTS profiler)
  if(ignition-common3_FOUND)
    add_definitions("-DIGN_PROFILER_ENABLE=1" "-DIGN_PROFILER_REMOTERY=1")
    message(STATUS "Profiler is active")
  else()
    message(WARNING "Can't find Ignition common3. Profiler will not be actived")
    add_definitions("-DIGN_PROFILER_ENABLE=0" "-DIGN_PROFILER_REMOTERY=0")
  endif()
endif()

################################################################################
# Build
################################################################################
include_directories(include
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS}
                 ${gazebo_dev_LIBRARY_DIRS}
)
# botanbot_ackermann_drive
add_library(botanbot_ackermann_drive SHARED
  plugins/botanbot_ackermann_drive.cpp
)
target_include_directories(botanbot_ackermann_drive PUBLIC include)
ament_target_dependencies(botanbot_ackermann_drive
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(botanbot_ackermann_drive)
if(ENABLE_PROFILER)
  target_include_directories(botanbot_ackermann_drive PUBLIC ${ignition-common3_INCLUDE_DIRS})
  target_link_libraries(botanbot_ackermann_drive ${ignition-common3_LIBRARIES})
endif()
#realsense_gazeo_plugin
add_library(
    realsense_gazebo_plugin SHARED
    plugins/RealSensePlugin.cpp
    plugins/gazebo_ros_realsense.cpp
    )
ament_target_dependencies(realsense_gazebo_plugin 
  "rclcpp"
  "gazebo_ros"
  "image_transport"
  "camera_info_manager"
  "sensor_msgs"
)
ament_export_libraries(realsense_gazebo_plugin)
target_link_libraries(realsense_gazebo_plugin ${GAZEBO_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS 
        botanbot_ackermann_drive
        realsense_gazebo_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)


install(DIRECTORY launch models worlds plugins
  DESTINATION share/${PROJECT_NAME}/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(botanbot)
ament_package()

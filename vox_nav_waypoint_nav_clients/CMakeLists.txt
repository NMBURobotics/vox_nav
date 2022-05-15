cmake_minimum_required(VERSION 3.5)
project(vox_nav_waypoint_nav_clients CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vox_nav_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(vox_nav_utilities REQUIRED)

set(dependencies
rclcpp
rclcpp_action
visualization_msgs
geometry_msgs
sensor_msgs
tf2_geometry_msgs
vox_nav_msgs
robot_localization
vox_nav_utilities
)

include_directories(include)

add_executable(navigate_through_poses_client src/navigate_through_poses_client.cpp)
ament_target_dependencies(navigate_through_poses_client ${dependencies})

add_executable(navigate_through_gps_poses_client src/navigate_through_gps_poses_client.cpp)
ament_target_dependencies(navigate_through_gps_poses_client ${dependencies})

install(TARGETS navigate_through_poses_client
                navigate_through_gps_poses_client
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
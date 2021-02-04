cmake_minimum_required(VERSION 3.5)
project(botanbot_to_pose_navigator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(botanbot_msgs REQUIRED)


include_directories(
  include)

set(dependencies
rclcpp
rclcpp_action
nav_msgs
tf2_ros
tf2_geometry_msgs
botanbot_msgs
)

set(gps_client_executable_name gps_waypoint_follower_demo)

add_executable(${gps_client_executable_name}
  src/gps_waypoint_follower_demo.cpp)


ament_target_dependencies(${gps_client_executable_name}
  ${dependencies})

target_link_libraries(${gps_client_executable_name})

set(gps_waypoint_collector_executable_name gps_waypoint_collector)

add_executable(${gps_waypoint_collector_executable_name}
  src/gps_waypoint_collector.cpp)
 
ament_target_dependencies(${gps_waypoint_collector_executable_name}
  ${dependencies})

target_link_libraries(${gps_waypoint_collector_executable_name} ${YAML_CPP_LIBRARIES})

install(TARGETS ${gps_client_executable_name} ${gps_waypoint_collector_executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(
  DIRECTORY launch params models worlds
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
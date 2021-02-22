cmake_minimum_required(VERSION 3.5)
project(botanbot_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(nav_msgs REQUIRED) 
find_package(octomap_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OrientedNavSatFix.msg"
  "srv/GetOctomap.srv"
  "action/ComputePathToPose.action"
  "action/FollowPath.action"
  "action/NavigateToPose.action"
  DEPENDENCIES 
    builtin_interfaces 
    geometry_msgs 
    std_msgs 
    nav_msgs
    sensor_msgs
    octomap_msgs
    rosidl_default_generators
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

cmake_minimum_required(VERSION 3.10)
project(vox_nav_drake_ros)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake_ros REQUIRED)
find_package(drake CONFIG REQUIRED PATHS /opt/drake)
find_package(rclcpp REQUIRED)
find_package(drake_ros_core REQUIRED)
find_package(test_msgs REQUIRED)

include_directories(
  include
 )

set(dependencies
    rclcpp
    drake 
    drake_ros_core
    test_msgs)

add_executable(drake_ros_node src/drake_ros_node.cc)
ament_target_dependencies(drake_ros_node ${dependencies})

install(TARGETS drake_ros_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY launch 
DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()

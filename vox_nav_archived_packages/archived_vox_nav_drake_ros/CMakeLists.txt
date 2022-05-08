cmake_minimum_required(VERSION 3.10)
project(archived_vox_nav_drake_ros)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake_ros REQUIRED)
find_package(drake CONFIG REQUIRED PATHS /opt/drake)
find_package(rclcpp REQUIRED)
find_package(drake_ros_core REQUIRED)
find_package(test_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
 )

set(dependencies
    rclcpp
    drake 
    drake_ros_core
    sensor_msgs
    geometry_msgs
    nav_msgs
    test_msgs
    tf2_ros
    tf2_geometry_msgs
    ament_index_cpp
    )

    
add_executable(lyapanuv_control src/lyapanuv_control.cc)
ament_target_dependencies(lyapanuv_control ${dependencies})

add_executable(sos_lyapunov_test src/sos_lyapunov_test.cc)
ament_target_dependencies(sos_lyapunov_test ${dependencies})

add_executable(read_modelurdf src/read_modelurdf.cc)
ament_target_dependencies(read_modelurdf ${dependencies})

install(TARGETS lyapanuv_control
                sos_lyapunov_test
                read_modelurdf
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY launch urdf meshes
DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()

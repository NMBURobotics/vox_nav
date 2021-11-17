cmake_minimum_required(VERSION 3.5)
project(vox_nav_cupoch_experimental)

# Add this repository's cmake modules to CMAKE_MODULE_PATH
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

include(ga_build_common)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(vox_nav_msgs REQUIRED)
find_package(vox_nav_utilities REQUIRED)
find_package(thrust REQUIRED)
find_package(cupoch_conversions REQUIRED)

message("zs: CUPOCH_INCLUDE_DIRS = " ${CUPOCH_INCLUDE_DIRS})
message("zs: CUPOCH_LIBRARIES = " ${CUPOCH_LIBRARIES})
message("zs: EIGEN3_INCLUDE_DIRS = " ${EIGEN3_INCLUDE_DIRS})

 
include_directories(include
                   ${PCL_INCLUDE_DIRS}
                   ${THRUST_INCLUDE_DIR}
                   ${CUPOCH_INCLUDE_DIRS}
                   ${CUDA_INCLUDE_DIRS})

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  Eigen3
  OCTOMAP
  octomap_msgs
  visualization_msgs
  vox_nav_msgs
  vox_nav_utilities
  cupoch_conversions
)

add_definitions(${PCL_DEFINITIONS})

set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")
APPEND_TARGET_ARCH_FLAGS()
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}
    ${CUPOCH_NVCC_FLAGS}
)
add_definitions(${CUPOCH_DEFINITIONS})
message("zs: ${CMAKE_PROJECT_NAME} CUDA_NVCC_FLAGS = " ${CUDA_NVCC_FLAGS})
include(GATest)

cuda_add_executable(ros2bag_kitti_bin  src/ros2bag_kitti_bin.cu)
ament_target_dependencies(ros2bag_kitti_bin ${dependencies})
target_include_directories(ros2bag_kitti_bin PUBLIC ${CUPOCH_INCLUDE_DIRS} ${THRUST_INCLUDE_DIR} ${CUDA_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_directories(ros2bag_kitti_bin PUBLIC ${PCL_LIBRARY_DIRS} ${CUPOCH_LIBRARY_DIRS} ${CUDA_LIBRARY_DIRS})
target_link_libraries(ros2bag_kitti_bin  ${PCL_LIBRARIES} ${CUPOCH_LIBRARIES} ${CUDA_LIBRARIES})

cuda_add_executable(cloud_segmentation  src/cloud_segmentation.cu)
ament_target_dependencies(cloud_segmentation ${dependencies})
target_include_directories(cloud_segmentation PUBLIC ${CUPOCH_INCLUDE_DIRS} ${THRUST_INCLUDE_DIR} ${CUDA_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_directories(cloud_segmentation PUBLIC ${PCL_LIBRARY_DIRS} ${CUPOCH_LIBRARY_DIRS} ${CUDA_LIBRARY_DIRS})
target_link_libraries(cloud_segmentation  ${PCL_LIBRARIES} ${CUPOCH_LIBRARIES} ${CUDA_LIBRARIES})

install(TARGETS ros2bag_kitti_bin
                cloud_segmentation
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/)

install(DIRECTORY launch cmake
        DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)

ament_package()

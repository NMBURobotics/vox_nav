cmake_minimum_required(VERSION 3.15)
project(vox_nav_misc)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
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
find_package(OpenMP REQUIRED)
find_package(fast_gicp REQUIRED)
find_package(CUDA REQUIRED)

include_directories(include
        ${PCL_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${fast_gicp_INCLUDE_DIRS})

set(dependencies
        rclcpp
        sensor_msgs
        geometry_msgs
        nav_msgs
        vision_msgs
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
        fast_gicp
        )
        
option(BUILD_VGICP_CUDA "Build GPU-powered VGICP" ON)
add_definitions(${PCL_DEFINITIONS})
add_definitions(-BUILD_VGICP_CUDA)

add_executable(naive_lidar_clustering src/naive_lidar_clustering.cpp)
ament_target_dependencies(naive_lidar_clustering ${dependencies})
target_include_directories(naive_lidar_clustering PUBLIC ${CUDA_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_directories(naive_lidar_clustering PUBLIC ${PCL_LIBRARY_DIRS} ${CUDA_LIBRARY_DIRS})
target_link_libraries(naive_lidar_clustering ${PCL_LIBRARIES} ${CUDA_LIBRARIES} OpenMP::OpenMP_CXX)

add_executable(pcl_cpu_ndt src/pcl_cpu_ndt.cpp)
ament_target_dependencies(pcl_cpu_ndt ${dependencies})
target_include_directories(pcl_cpu_ndt PUBLIC ${CUDA_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_directories(pcl_cpu_ndt PUBLIC ${PCL_LIBRARY_DIRS} ${CUDA_LIBRARY_DIRS})
target_link_libraries(pcl_cpu_ndt ${PCL_LIBRARIES} ${CUDA_LIBRARIES} OpenMP::OpenMP_CXX)

add_executable(ukf_tracker src/ukf_tracker.cpp)
ament_target_dependencies(ukf_tracker ${dependencies})

cuda_add_executable(fast_gicp_client src/fast_gicp_client.cpp)
ament_target_dependencies(fast_gicp_client ${dependencies})
target_include_directories(fast_gicp_client PUBLIC ${CUDA_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${fast_gicp_INCLUDE_DIRS})
target_link_directories(fast_gicp_client PUBLIC ${PCL_LIBRARY_DIRS} ${CUDA_LIBRARY_DIRS})
target_link_libraries(fast_gicp_client ${PCL_LIBRARIES} ${CUDA_LIBRARIES} ${fast_gicp_LIBRARIES} OpenMP::OpenMP_CXX)

install(TARGETS naive_lidar_clustering
                pcl_cpu_ndt
                ukf_tracker
                fast_gicp_client
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)

ament_package()

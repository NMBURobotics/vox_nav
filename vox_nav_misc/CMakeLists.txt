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
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include
        ${PCL_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${fast_gicp_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        )

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
        cv_bridge
        )
        
option(BUILD_VGICP_CUDA "Build GPU-powered VGICP" ON)
add_definitions(${PCL_DEFINITIONS})
add_definitions(-BUILD_VGICP_CUDA)

# ICP AND NDT LOCALIZATION NODES
cuda_add_executable(fast_gicp_client src/fast_gicp_client.cpp)
ament_target_dependencies(fast_gicp_client ${dependencies})
target_include_directories(fast_gicp_client PUBLIC ${CUDA_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${fast_gicp_INCLUDE_DIRS})
target_link_directories(fast_gicp_client PUBLIC ${PCL_LIBRARY_DIRS} ${CUDA_LIBRARY_DIRS})
target_link_libraries(fast_gicp_client ${PCL_LIBRARIES} ${CUDA_LIBRARIES} ${fast_gicp_LIBRARIES} OpenMP::OpenMP_CXX)

cuda_add_executable(fast_gicp_client_no_gps src/fast_gicp_client_no_gps.cpp)
ament_target_dependencies(fast_gicp_client_no_gps ${dependencies})
target_include_directories(fast_gicp_client_no_gps PUBLIC ${CUDA_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${fast_gicp_INCLUDE_DIRS})
target_link_directories(fast_gicp_client_no_gps PUBLIC ${PCL_LIBRARY_DIRS} ${CUDA_LIBRARY_DIRS})
target_link_libraries(fast_gicp_client_no_gps ${PCL_LIBRARIES} ${CUDA_LIBRARIES} ${fast_gicp_LIBRARIES} OpenMP::OpenMP_CXX)

add_executable(pcl_cpu_ndt src/pcl_cpu_ndt.cpp)
ament_target_dependencies(pcl_cpu_ndt ${dependencies})
target_include_directories(pcl_cpu_ndt PUBLIC ${CUDA_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_directories(pcl_cpu_ndt PUBLIC ${PCL_LIBRARY_DIRS} ${CUDA_LIBRARY_DIRS})
target_link_libraries(pcl_cpu_ndt ${PCL_LIBRARIES} ${CUDA_LIBRARIES} OpenMP::OpenMP_CXX)

# CLUSTERING AND TRACKING NODES
add_executable(naive_lidar_clustering src/naive_lidar_clustering.cpp)
ament_target_dependencies(naive_lidar_clustering ${dependencies})
target_include_directories(naive_lidar_clustering PUBLIC ${CUDA_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_directories(naive_lidar_clustering PUBLIC ${PCL_LIBRARY_DIRS} ${CUDA_LIBRARY_DIRS})
target_link_libraries(naive_lidar_clustering ${PCL_LIBRARIES} ${CUDA_LIBRARIES} OpenMP::OpenMP_CXX)

add_executable(ukf_tracker src/ukf_tracker.cpp)
ament_target_dependencies(ukf_tracker ${dependencies})

# TRULY MISC NODES
add_executable(traversablity_estimator src/traversablity_estimator.cpp)
ament_target_dependencies(traversablity_estimator ${dependencies})
target_link_libraries(traversablity_estimator ${PCL_LIBRARIES})

add_executable(lidar_rgb_image_fuser src/lidar_rgb_image_fuser.cpp)
ament_target_dependencies(lidar_rgb_image_fuser ${dependencies})
target_include_directories(lidar_rgb_image_fuser PUBLIC ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )
target_link_libraries(lidar_rgb_image_fuser ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(stick_imu_to_inertial_frame src/stick_imu_to_inertial_frame.cpp)
ament_target_dependencies(stick_imu_to_inertial_frame ${dependencies})
target_include_directories(stick_imu_to_inertial_frame PUBLIC ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )
target_link_libraries(stick_imu_to_inertial_frame ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# TOOLS
add_executable(fix_ouster_direction_node tools/fix_ouster_direction_node.cpp)
ament_target_dependencies(fix_ouster_direction_node ${dependencies})
target_include_directories(fix_ouster_direction_node PUBLIC ${PCL_INCLUDE_DIRS})

add_executable(fix_ouster_pointtype_node tools/fix_ouster_pointtype_node.cpp)
ament_target_dependencies(fix_ouster_pointtype_node ${dependencies})
target_include_directories(fix_ouster_pointtype_node PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(fix_ouster_pointtype_node ${PCL_LIBRARIES})

add_executable(traversablity_integrator_node tools/traversablity_integrator_node.cpp)
ament_target_dependencies(traversablity_integrator_node ${dependencies})
target_include_directories(traversablity_integrator_node PUBLIC ${PCL_INCLUDE_DIRS})

install(TARGETS   fast_gicp_client
                  fast_gicp_client_no_gps
                  pcl_cpu_ndt
                  naive_lidar_clustering
                  ukf_tracker
                  traversablity_estimator
                  lidar_rgb_image_fuser
                  stick_imu_to_inertial_frame
                  fix_ouster_direction_node
                  fix_ouster_pointtype_node
                  traversablity_integrator_node
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)

ament_package()

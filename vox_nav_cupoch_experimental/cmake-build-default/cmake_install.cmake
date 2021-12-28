# Install script for directory: /home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_INSTALL_DEFAULT_DIRECTORY_PERMISSIONS)
  set(CMAKE_INSTALL_DEFAULT_DIRECTORY_PERMISSIONS "OWNER_READ;OWNER_WRITE;OWNER_EXECUTE;GROUP_READ;GROUP_EXECUTE;WORLD_READ;WORLD_EXECUTE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/ros2bag_kitti_bin" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/ros2bag_kitti_bin")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/ros2bag_kitti_bin"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental" TYPE EXECUTABLE FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/ros2bag_kitti_bin")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/ros2bag_kitti_bin" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/ros2bag_kitti_bin")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/ros2bag_kitti_bin"
         OLD_RPATH "/home/atas/colcon_ws/install/cupoch/lib:/home/atas/colcon_ws/install/pcl_ros/lib:/opt/ros/foxy/lib:/home/atas/colcon_ws/install/vox_nav_msgs/lib:/home/atas/colcon_ws/install/vox_nav_utilities/lib:/usr/local/cuda-11.4/lib64:/home/atas/colcon_ws/install/ApproxMVBB/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/ros2bag_kitti_bin")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/cloud_segmentation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/cloud_segmentation")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/cloud_segmentation"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental" TYPE EXECUTABLE FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/cloud_segmentation")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/cloud_segmentation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/cloud_segmentation")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/cloud_segmentation"
         OLD_RPATH "/home/atas/colcon_ws/install/cupoch/lib:/home/atas/colcon_ws/install/pcl_ros/lib:/opt/ros/foxy/lib:/home/atas/colcon_ws/install/vox_nav_msgs/lib:/home/atas/colcon_ws/install/vox_nav_utilities/lib:/usr/local/cuda-11.4/lib64:/home/atas/colcon_ws/install/ApproxMVBB/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/cloud_segmentation")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/cloud_clustering" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/cloud_clustering")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/cloud_clustering"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental" TYPE EXECUTABLE FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/cloud_clustering")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/cloud_clustering" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/cloud_clustering")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/cloud_clustering"
         OLD_RPATH "/home/atas/colcon_ws/install/cupoch/lib:/home/atas/colcon_ws/install/pcl_ros/lib:/opt/ros/foxy/lib:/home/atas/colcon_ws/install/vox_nav_msgs/lib:/home/atas/colcon_ws/install/vox_nav_utilities/lib:/usr/local/cuda-11.4/lib64:/home/atas/colcon_ws/install/ApproxMVBB/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vox_nav_cupoch_experimental/cloud_clustering")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/" TYPE DIRECTORY FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vox_nav_cupoch_experimental" TYPE DIRECTORY FILES
    "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/launch"
    "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake"
    "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/config"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/vox_nav_cupoch_experimental")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/vox_nav_cupoch_experimental")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vox_nav_cupoch_experimental/environment" TYPE FILE FILES "/opt/ros/foxy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vox_nav_cupoch_experimental/environment" TYPE FILE FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vox_nav_cupoch_experimental/environment" TYPE FILE FILES "/opt/ros/foxy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vox_nav_cupoch_experimental/environment" TYPE FILE FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vox_nav_cupoch_experimental" TYPE FILE FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vox_nav_cupoch_experimental" TYPE FILE FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vox_nav_cupoch_experimental" TYPE FILE FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vox_nav_cupoch_experimental" TYPE FILE FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vox_nav_cupoch_experimental" TYPE FILE FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/ament_cmake_index/share/ament_index/resource_index/packages/vox_nav_cupoch_experimental")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vox_nav_cupoch_experimental/cmake" TYPE FILE FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vox_nav_cupoch_experimental/cmake" TYPE FILE FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vox_nav_cupoch_experimental/cmake" TYPE FILE FILES
    "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/ament_cmake_core/vox_nav_cupoch_experimentalConfig.cmake"
    "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/ament_cmake_core/vox_nav_cupoch_experimentalConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vox_nav_cupoch_experimental" TYPE FILE FILES "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/atas/colcon_ws/src/vox_nav/vox_nav_cupoch_experimental/cmake-build-default/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")

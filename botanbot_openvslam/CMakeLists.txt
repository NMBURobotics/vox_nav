cmake_minimum_required(VERSION 3.5)
project(botanbot_openvslam LANGUAGES CXX C)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
#################
# Find packages #
#################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# thread library
find_package(Threads REQUIRED)

# OpenVSLAM
find_library(OpenVSLAM_LIB openvslam HINTS ${OpenVSLAM_LIB_DIR})

# OpenCV
find_package(OpenCV 3.3.1 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 4.0)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV >= 3.3.1 not found")
    endif()
endif()
message(STATUS "Use OpenCV ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

# Eigen
find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# spdlog
set(spdlog_INCLUDE_DIR ${OpenVSLAM_ROOT}/3rd/spdlog/include)
include_directories(${spdlog_INCLUDE_DIR})

# popl
set(popl_INCLUDE_DIR ${OpenVSLAM_ROOT}/3rd/popl/include)
include_directories(${popl_INCLUDE_DIR})

# json
set(json_INCLUDE_DIR ${OpenVSLAM_ROOT}/3rd/json/include)
include_directories(${json_INCLUDE_DIR})

# pangolin viewer
set(USE_PANGOLIN_VIEWER ON CACHE BOOL "Enable Pangolin Viewer")
if(USE_PANGOLIN_VIEWER)
    find_library(Pangolin_Viewer_LIB pangolin_viewer HINTS ${OpenVSLAM_LIB_DIR})
    find_package(Pangolin REQUIRED)
endif()

#################
# BoW framework #
#################

set(BOW_FRAMEWORK "DBoW2" CACHE STRING "DBoW2 or FBoW")
set_property(CACHE BOW_FRAMEWORK PROPERTY STRINGS "DBoW2" "FBoW")

if(BOW_FRAMEWORK MATCHES "DBoW2")
    find_package(DBoW2 REQUIRED)
    set(BOW_INCLUDE_DIRS ${DBoW2_INCLUDE_DIRS})
    set(BOW_LIBS ${DBoW2_LIBS})
    add_definitions(-DUSE_DBOW2)
elseif(BOW_FRAMEWORK MATCHES "FBoW")
    find_package(fbow REQUIRED)
    set(BOW_INCLUDE_DIRS ${fbow_INCLUDE_DIRS})
    set(BOW_LIBS ${fbow_LIBS})
else()
    message(FATAL_ERROR "Invalid BoW framework: ${BOW_FRAMEWORK}")
endif()
include_directories(${BOW_INCLUDE_DIRS})
message(STATUS "BoW framework: ${BOW_FRAMEWORK} (found in ${BOW_INCLUDE_DIRS})")

#########
# Build #
#########

if(USE_PANGOLIN_VIEWER)
    message(STATUS "Viewer for examples: pangolin viewer")
endif()

set(EXECUTABLE_TARGETS "")

add_executable(run_slam src/run_slam.cc)
list(APPEND EXECUTABLE_TARGETS run_slam)

add_executable(run_localization src/run_localization.cc)
list(APPEND EXECUTABLE_TARGETS run_localization)

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    # pangolin_viewer is used on a priority basis
    if(USE_PANGOLIN_VIEWER)
        # set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE -DUSE_PANGOLIN_VIEWER)
        # link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                ${Pangolin_Viewer_LIB} ${Pangolin_LIBRARIES})
    endif()

    # build
    target_include_directories(${EXECUTABLE_TARGET}
                               PRIVATE ${OpenVSLAM_SRC_DIR} ${rclcpp_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)
    target_link_libraries(${EXECUTABLE_TARGET}
                          PRIVATE ${OpenVSLAM_LIB} ${rclcpp_LIBRARIES} ${OpenCV_LIBRARIES})
    ament_target_dependencies(${EXECUTABLE_TARGET}
                              PUBLIC rclcpp cv_bridge image_transport rcutils)
    install(TARGETS ${EXECUTABLE_TARGET}
            DESTINATION lib/${PROJECT_NAME})
endforeach()
ament_export_include_directories(include)
ament_package()

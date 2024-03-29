cmake_minimum_required(VERSION 3.5)
project(archived_vox_nav_openvslam LANGUAGES CXX C)

if(POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW)
endif()
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 OLD)
endif()

set(OpenVSLAM_ROOT "~/openvslam/" CACHE STRING "Root directory of OpenVSLAM")
message(STATUS "Root directory of OpenVSLAM: ${OpenVSLAM_ROOT}")

set(OpenVSLAM_SRC_DIR ${OpenVSLAM_ROOT}/src)
message(STATUS "Source directory of OpenVSLAM: ${OpenVSLAM_SRC_DIR}")
set(OpenVSLAM_LIB_DIR ${OpenVSLAM_ROOT}/build/lib CACHE STRING "Directory which contains the shared library of OpenVSLAM")
message(STATUS "Library directory of OpenVSLAM: ${OpenVSLAM_LIB_DIR}")

list(APPEND CMAKE_MODULE_PATH ${OpenVSLAM_ROOT}/cmake)

##################
# Set build type #
##################

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

###########################
# Set application options #
###########################

set(USE_SANITIZER OFF CACHE BOOL "Enable Address/Memory sanitizer (set env as ASAN_OPTIONS=detect_leaks=1)")

if(USE_SANITIZER)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
    message(STATUS "Address/Memory sanitizer: ENABLED")
else()
    message(STATUS "Address/Memory sanitizer: DISABLED")
endif()

set(USE_STACK_TRACE_LOGGER OFF CACHE BOOL "Enable automatic stack trace logger of google-glog")

if(USE_STACK_TRACE_LOGGER)
    message(STATUS "Stack trace logger: ENABLED")
    # Glog
    find_package(Glog REQUIRED)
    include_directories(${GLOG_INCLUDE_DIR})
else()
    message(STATUS "Stack trace logger: DISABLED")
endif()

set(USE_GOOGLE_PERFTOOLS OFF CACHE BOOL "Enable profiler of google-perftools")

if(USE_GOOGLE_PERFTOOLS)
    message(STATUS "Google Perftools: ENABLED")
    # Gperftools
    find_package(Gperftools REQUIRED)
    include_directories(${GPERFTOOLS_INCLUDE_DIRS})
else()
    message(STATUS "Google Perftools: DISABLED")
endif()

########################
# Set compiler options #
########################

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Og")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og")

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -ffast-math")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -ffast-math")

set(BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "Enable architecture-aware optimization (-march=native)")

if(BUILD_WITH_MARCH_NATIVE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mtune=native -march=native")
    set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -mtune=native -march=native")
    message(STATUS "Architecture-aware optimization (-march=native): ENABLED")
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mtune=native")
    set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -mtune=native")
    message(STATUS "Architecture-aware optimization (-march=native): DISABLED")
endif()

#######################
# Check C++11 support #
#######################

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    message(STATUS "C++11 support: OK (-std=c++17)")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    message(STATUS "C++11 support: OK (-std=c++14)")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    message(STATUS "C++11 support: OK (-std=c++11)")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    message(STATUS "C++11 support: OK (-std=c++0x)")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#################
# Find packages #
#################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(vox_nav_utilities REQUIRED)
find_package(vox_nav_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(yaml-cpp REQUIRED )
find_package(Threads REQUIRED)
find_library(OpenVSLAM_LIB openvslam HINTS ${OpenVSLAM_LIB_DIR})
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

set(spdlog_INCLUDE_DIR ${OpenVSLAM_ROOT}/3rd/spdlog/include)
set(popl_INCLUDE_DIR ${OpenVSLAM_ROOT}/3rd/popl/include)
set(json_INCLUDE_DIR ${OpenVSLAM_ROOT}/3rd/json/include)
 
include_directories(include
                    ${OpenVSLAM_SRC_DIR}
                    ${rclcpp_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${spdlog_INCLUDE_DIR}
                    ${popl_INCLUDE_DIR}
                    ${json_INCLUDE_DIR}
                    ${OpenCV_INCLUDE_DIRS}
                    ${YAML_CPP_INCLUDE_DIR})

set(dependencies
rclcpp 
cv_bridge 
image_transport
rcutils 
vox_nav_utilities
vox_nav_msgs
nav_msgs
tf2_geometry_msgs
tf2_ros
geometry_msgs
robot_localization
)

# pangolin viewer
set(USE_PANGOLIN_VIEWER ON CACHE BOOL "Enable Pangolin Viewer")
if(USE_PANGOLIN_VIEWER)
    find_library(Pangolin_Viewer_LIB pangolin_viewer HINTS ${OpenVSLAM_LIB_DIR})
    find_package(Pangolin REQUIRED)
endif()

# socket publisher
set(USE_SOCKET_PUBLISHER OFF CACHE BOOL "Enable Socket Publisher")
if(USE_SOCKET_PUBLISHER)
    find_package(sioclient REQUIRED)
    find_package(Protobuf REQUIRED)
    find_library(Socket_Publisher_LIB socket_publisher HINTS ${OpenVSLAM_LIB_DIR})
    if(NOT PROTOBUF_PROTOC_EXECUTABLE)
        message(FATAL_ERROR "Could not find protoc executable (PROTOBUF_PROTOC_EXECUTABLE)")
    endif()
    message(STATUS "Found protoc executable: ${PROTOBUF_PROTOC_EXECUTABLE}")
    include_directories(${SIOCLIENT_INCLUDE_DIR} ${PROTOBUF_INCLUDE_DIRS})
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
elseif(USE_SOCKET_PUBLISHER)
    message(STATUS "Viewer for examples: socket publisher")
else()
    message(STATUS "Viewer for examples: none")
endif()

set(EXECUTABLE_TARGETS "")

add_executable(run_slam src/run_slam.cpp src/gps_data_handler.cpp)
list(APPEND EXECUTABLE_TARGETS run_slam)

add_executable(run_localization src/run_localization.cpp src/gps_data_handler.cpp)
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
    elseif(USE_SOCKET_PUBLISHER)
        # set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE -DUSE_SOCKET_PUBLISHER)
        # link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                ${Socket_Publisher_LIB} ${SIOCLIENT_LIBRARY} ${PROTOBUF_LIBRARIES})
    endif()

    # setup stack trace logger
    if(USE_STACK_TRACE_LOGGER)
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE -DUSE_STACK_TRACE_LOGGER)
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                ${GLOG_LIBRARIES})
    endif()

    # setup google-perftools
    if(USE_GOOGLE_PERFTOOLS)
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE -DUSE_GOOGLE_PERFTOOLS)
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                ${GPERFTOOLS_LIBRARIES})
    endif()

    target_link_libraries(${EXECUTABLE_TARGET}
                          PRIVATE ${OpenVSLAM_LIB} ${rclcpp_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})
    ament_target_dependencies(${EXECUTABLE_TARGET} PUBLIC ${dependencies})
    install(TARGETS ${EXECUTABLE_TARGET}
            DESTINATION lib/${PROJECT_NAME})
endforeach()

install(
  DIRECTORY config launch rviz scripts
  DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_package()

# general config
set(GA_BUILD_TEST ON)

# root dirs
set(CUPOCH_ROOT $ENV{HOME}/colcon_ws/install/cupoch)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Add this repository's cmake modules to CMAKE_MODULE_PATH
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR})
# 优先查找，增加对特定版本的支持，解注释后要清除重新编译
# 增加对opencv4支持
# 单独在各个项目中list, 公共部分使用opencv3
# list(INSERT CMAKE_PREFIX_PATH 0 /opt/opencv/opencv-4.3/)
# list(INSERT CMAKE_PREFIX_PATH 0 /opt/opencv/opencv-4.2/)
# list(INSERT CMAKE_PREFIX_PATH 0 /opt/opencv/opencv-4.3.0/)
# list(INSERT CMAKE_PREFIX_PATH 0 /opt/opencv/opencv-4.2.0/)
# list(INSERT CMAKE_PREFIX_PATH 0 /opt/opencv/opencv-4.5.1/)
# 增加对libtorch的支持
list(INSERT CMAKE_PREFIX_PATH 0 $ENV{HOME}/opt/libtorch)
# 增加对caffe的支持
list(INSERT CMAKE_PREFIX_PATH 0 /opt/caffe/caffe-mobilenet-ssd)
# 增加对pcl的支持
# 用默认的pcl1.8
# list(INSERT CMAKE_PREFIX_PATH 0 /opt/pcl/pcl-1.9.1)
# list(INSERT CMAKE_PREFIX_PATH 0 /opt/pcl/pcl-1.10.1)
# 增加对open3d的支持
# list(INSERT CMAKE_PREFIX_PATH 0 /opt/open3d/open3d-gpu)
# list(INSERT CMAKE_PREFIX_PATH 0 /opt/open3d/open3d)
# 使用自带eigen
list(INSERT CMAKE_PREFIX_PATH 0 /opt/eigen/eigen)
# 增加对cupoch支持
find_package(PkgConfig QUIET)
find_package(pybind11  QUIET)
find_package(OpenGL    QUIET)

include(GenerateExportHeader)
if (PKGCONFIG_FOUND)
    pkg_search_module(EIGEN3          eigen3>=3.2.7   QUIET)
    pkg_search_module(GLFW            glfw3           QUIET)
    pkg_search_module(GLEW            glew            QUIET)
    pkg_search_module(imgui           imgui           QUIET)
    pkg_search_module(JSONCPP         jsoncpp>=1.7.0  QUIET)
    pkg_search_module(PNG             libpng>=1.6.0   QUIET)
    pkg_search_module(JPEG_TURBO      libturbojpeg    QUIET)
endif (PKGCONFIG_FOUND)
set(CUPOCH_INCLUDE_DIRS ${CUPOCH_ROOT}/include
    ${CUPOCH_ROOT}/third_party
    ${CUPOCH_ROOT}/third_party/rmm/include
    ${CUPOCH_ROOT}/third_party/cnmem/include
    ${CUPOCH_ROOT}/third_party/fmt/include
    ${CUPOCH_ROOT}/third_party/liblzf/include
    ${CUPOCH_ROOT}/third_party/eigen/
    ${CUPOCH_ROOT}/third_party/spdlog/include
    ${PNG_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${imgui_INCLUDE_DIRS}
)
set(CUPOCH_LIBRARY_DIRS ${CUPOCH_ROOT}/lib)
set(CUPOCH_LIBRARIES
    cupoch_camera
    cupoch_collision
    cupoch_geometry
    cupoch_integration
    cupoch_io
    cupoch_odometry
    cupoch_planning
    cupoch_registration
    cupoch_utility
    cupoch_visualization
    tinyobjloader
    turbojpeg
    stdgpu
    flann_cuda_s
    # rmm
    liblzf
    rply
    spdlog
    imgui
    ${spdlog_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${PNG_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${GLFW_LIBRARIES}
    ${OPENGL_gl_LIBRARY}
    ${imgui_LIBRARIES}
)

find_package(CUDA REQUIRED  QUIET)
find_package(Eigen3 REQUIRED  QUIET)
set(CMAKE_CUDA_COMPILE_FEATURES cuda_std_14)
set(CUPOCH_NVCC_FLAGS
    -arch=sm_75
    --expt-relaxed-constexpr
    --expt-extended-lambda
    --default-stream per-thread
    --use_fast_math
    -Xcudafe "--diag_suppress=integer_sign_change"
    -Xcudafe "--diag_suppress=partial_override"
    -Xcudafe "--diag_suppress=virtual_function_decl_hidden")
if(CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" )
    list(APPEND CUPOCH_NVCC_FLAGS
        -G;-g
    )
endif()
set(CUPOCH_DEFINITIONS
    -DFLANN_USE_CUDA
    -DUSE_RMM
)
link_directories(${CUPOCH_LIBRARY_DIRS})

include(GAUtils)
include(CudaComputeTargetFlags)

# default rel type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to Release as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE
        STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()
set(TARGET_ARCH ${CMAKE_SYSTEM_PROCESSOR})
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Clang tidy
if(TIDY_WITH_CLANG)
  string(CONCAT CMAKE_CXX_CLANG_TIDY
    "clang-tidy;"
    "-checks=-*,"
    "bugprone-*,"
    "cert-*,"
    "cppcoreguidelines-*,"
    "clang-analyze-*,"
    "google-*,"
    "hicpp-*,"
    "modernize-*,"
    "performance-*,"
    "readability-*")
  message(${CMAKE_CXX_CLANG_TIDY})
endif()
# enable c++14
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fPIC -O3 -Wno-write-strings")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,$ORIGIN")
if (NOT BUILD_SHARED_LIBS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden -fvisibility-inlines-hidden")
endif (NOT BUILD_SHARED_LIBS)
add_compile_options(-Wno-deprecated-declarations)
# In Release build -O3 will be added automatically by CMake
# We still enable -O3 at debug build to optimize performance
if (uppercase_CMAKE_BUILD_TYPE STREQUAL "DEBUG")
    add_definitions(-O3)
endif()
# Before CMake 3.14 setting CMAKE_POSITION_INDEPENDENT_CODE did not set the
# "-pie" flag for GCC or Clang
if("${CMAKE_VERSION}" VERSION_LESS "3.14.0")
    if ("${CMAKE_C_COMPILER_ID}" STREQUAL "Clang" OR "${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie")
    endif()
endif()
option(COVERAGE_ENABLED "Enable code coverage" FALSE)
if(COVERAGE_ENABLED)
    add_compile_options(--coverage)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
endif()
# Turn on compiler flags for our code
include(GACompilerFlags)


# cpack
find_program(LSB_RELEASE_EXEC lsb_release)
# Ubuntu
execute_process(COMMAND ${LSB_RELEASE_EXEC} -is
    OUTPUT_VARIABLE LSB_DISTRIBUTE_ID_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(COMMAND ${LSB_RELEASE_EXEC} -rs
    OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(COMMAND ${LSB_RELEASE_EXEC} -cs
    OUTPUT_VARIABLE LSB_CODE_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Packaging support
# ament
set(CATKIN_BUILD_BINARY_PACKAGE true)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_BINDIR lib/${PROJECT_NAME})
set(CMAKE_INSTALL_INCLUDEDIR include/${PROJECT_NAME})
set(CMAKE_INSTALL_SHAREDIR share/${PROJECT_NAME})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../)
# set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/ros/dashing")
# set(CPACK_SET_DESTDIR "/opt/ros/dashing/")

set(CMAKE_INSTALL_DEFAULT_DIRECTORY_PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ             GROUP_EXECUTE
    WORLD_READ             WORLD_EXECUTE)
set(CPACK_PACKAGE_VENDOR "GATeam")
set(CPACK_PACKAGE_CONTACT "lizhensheng03")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "C/C++ SDK for GA")
set(CPACK_PACKAGE_DESCRIPTION "C/C++ SDK for GA")
set(CPACK_PACKAGE_VERSION_MAJOR ${GA_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${GA_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${GA_VERSION_PATCH})
# set(CPACK_PACKAGE_VERSION ${GA_VERSION_STR})
# set(CPACK_PACKAGE_VERSION ${TODAY}-${GA_COMMIT_ID}-${LSB_RELEASE_ID_SHORT}-${TARGET_ARCH}-${CMAKE_BUILD_TYPE})
set(CPACK_PACKAGE_VERSION ${TODAY}-${GA_COMMIT_ID})
set(CPACK_PACKAGE_NAME "ros-${GA_ROS_VERSION}-${PROJECT_NAME}")
# set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${GA_COMMIT_ID}.${LSB_DISTRIBUTE_ID_SHORT}.${LSB_RELEASE_ID_SHORT}.${TARGET_ARCH}.${CMAKE_BUILD_TYPE}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${TODAY}-${GA_COMMIT_ID}-CUDA${CUDA_VERSION}-${LSB_RELEASE_ID_SHORT}.${TARGET_ARCH}.${CMAKE_BUILD_TYPE}")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_COMPONENTS_GROUPING "ONE_PER_GROUP")
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})

function(ga_install target)
    install(TARGETS ${target}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
endfunction()
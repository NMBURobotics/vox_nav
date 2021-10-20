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
include(GenerateExportHeader)
if (PKGCONFIG_FOUND)
    pkg_search_module(EIGEN3          eigen3>=3.2.7   QUIET)
    pkg_search_module(GLFW            glfw3           QUIET)
    pkg_search_module(GLEW            glew            QUIET)
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
    ${spdlog_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${CUDA_CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${PNG_LIBRARIES}
    ${JSONCPP_LIBRARIES}
)
set(CUPOCH_NVCC_FLAGS
    --expt-relaxed-constexpr
    --expt-extended-lambda
    --default-stream per-thread
    --use_fast_math
    -Xcudafe "--diag_suppress=integer_sign_change"
    -Xcudafe "--diag_suppress=partial_override"
    -Xcudafe "--diag_suppress=virtual_function_decl_hidden"
)
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
if (${LSB_CODE_SHORT} STREQUAL "bionic")
    set(GA_ROS_VERSION "dashing")
else()
    set(GA_ROS_VERSION "foxy")
endif()

set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STREQUAL "dashing")
    add_definitions(-DROS_DISTRO_DASHING)
elseif(${ROS_DISTRO} STREQUAL "foxy")
    add_definitions(-DROS_DISTRO_FOXY)
endif()

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
# include(CPack)
# cpack_add_component(
#     runtime
#     DISPLAY_NAME
#         Runtime
#     DESCRIPTION
#         "Dynamic Libraries for GA Runtime"
#     REQUIRED
# )
# cpack_add_component(
#     development
#     DISPLAY_NAME
#         Development
#     DESCRIPTION
#         "Headers and cmake files needed for GA Development"
#     REQUIRED
#     DEPENDS
#         runtime
# )
# cpack_add_component(
#     tools
#     DISPLAY_NAME
#         Tools
#     DESCRIPTION
#         "Tools for GA Development"
#     REQUIRED
#     DEPENDS
#         runtime
# )

message(STATUS "CUDA compilation status: $ENV{GPUAC_COMPILE_WITH_CUDA}.")

macro(GA_CHECK_CUDA)
  if ($ENV{GPUAC_COMPILE_WITH_CUDA})
    find_package(CUDA REQUIRED  QUIET)
    find_package(Eigen3 REQUIRED  QUIET)

    # if(NOT ${CUDA_VERSION} VERSION_LESS "10.0"
    #         AND NOT ${CUDA_VERSION} VERSION_EQUAL "10.0" )
    #   message(FATAL_ERROR "GPU support on Melodic requires CUDA<=10.0")
    # endif()
    if(${CUDA_VERSION} VERSION_GREATER "9.1"
          AND ${CMAKE_VERSION} VERSION_LESS "3.12.3")
      unset(CUDA_cublas_device_LIBRARY CACHE)
      set(CUDA_cublas_device_LIBRARY ${CUDA_cublas_LIBRARY})
      set(CUDA_CUBLAS_LIBRARIES ${CUDA_cublas_LIBRARY})
    endif()
    # if ("$ENV{ROS_DISTRO}" STREQUAL "melodic" AND ${EIGEN3_VERSION_STRING} VERSION_LESS "3.3.7")
    #   message(FATAL_ERROR "GPU support on Melodic requires Eigen version>= 3.3.7")
    # endif()
    if(NOT DEFINED CMAKE_CUDA_STANDARD)
        set(CMAKE_CUDA_STANDARD 14)
        set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    endif()
    set(USE_CUDA ON)
  else()
    message(WARNING "CUDA support is disabled. Set the GPUAC_COMPILE_WITH_CUDA environment variable and recompile to enable it")
    set(USE_CUDA OFF)
  endif()
endmacro()

# Try to adhere to strict ISO C++ as much as possible:
#    from https://lefticus.gitbooks.io/cpp-best-practices/content/02-Use_the_Tools_Available.html
function(autoware_set_compile_options target)
if(WIN32)
    # Causes the visibility macros to use dllexport rather than dllimport,
    # which is appropriate when building the dll but not consuming it.
    string(TOUPPER ${target} PROJECT_NAME_UPPER)
    target_compile_definitions(${target} PRIVATE ${PROJECT_NAME_UPPER}_BUILDING_DLL)
    target_compile_options(${target} PRIVATE "/bigobj")
    add_definitions(-D_CRT_NONSTDC_NO_WARNINGS)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    add_definitions(-D_WINSOCK_DEPRECATED_NO_WARNINGS)
else()
    target_compile_options(${target} PRIVATE
        -Wall
        -Werror
        -Wextra
        #-Wshadow             # causes issues with ROS 2 headers
        #-Wnon-virtual-dtor   # causes issues with ROS 2 headers
        -pedantic
        -Wcast-align
        -Wunused
        -Wconversion
        -Wsign-conversion
        -Wdouble-promotion
        #-Wnull-dereference    # gcc6
        #-Wduplicated-branches # gcc7
        #-Wduplicated-cond     # gcc6
        #-Wrestrict            # gcc7
        -fvisibility=hidden)
    # C++-only options
    target_compile_options(${target}
        PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -Woverloaded-virtual -Wold-style-cast>)

    if(NOT APPLE)
        # GCC/G++ Only, not CLang
        target_compile_options(${target}
            PUBLIC $<$<COMPILE_LANGUAGE:CXX>: -Wuseless-cast>)
        target_compile_options(${target} PRIVATE -Wlogical-op -frecord-gcc-switches)
    endif()

    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set_target_properties(${target} PROPERTIES COMPILE_FLAGS "-Og")
    else()
        set_target_properties(${target} PROPERTIES COMPILE_FLAGS "-O3 -ftree-vectorize")
    endif()
endif()
endfunction()


function(ga_install target)
    install(TARGETS ${target}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
endfunction()
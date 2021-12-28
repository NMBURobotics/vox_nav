# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_vox_nav_cupoch_experimental_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED vox_nav_cupoch_experimental_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(vox_nav_cupoch_experimental_FOUND FALSE)
  elseif(NOT vox_nav_cupoch_experimental_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(vox_nav_cupoch_experimental_FOUND FALSE)
  endif()
  return()
endif()
set(_vox_nav_cupoch_experimental_CONFIG_INCLUDED TRUE)

# output package information
if(NOT vox_nav_cupoch_experimental_FIND_QUIETLY)
  message(STATUS "Found vox_nav_cupoch_experimental: 0.0.0 (${vox_nav_cupoch_experimental_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'vox_nav_cupoch_experimental' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${vox_nav_cupoch_experimental_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(vox_nav_cupoch_experimental_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake")
foreach(_extra ${_extras})
  include("${vox_nav_cupoch_experimental_DIR}/${_extra}")
endforeach()

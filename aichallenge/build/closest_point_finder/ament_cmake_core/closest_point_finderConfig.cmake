# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_closest_point_finder_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED closest_point_finder_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(closest_point_finder_FOUND FALSE)
  elseif(NOT closest_point_finder_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(closest_point_finder_FOUND FALSE)
  endif()
  return()
endif()
set(_closest_point_finder_CONFIG_INCLUDED TRUE)

# output package information
if(NOT closest_point_finder_FIND_QUIETLY)
  message(STATUS "Found closest_point_finder: 0.0.0 (${closest_point_finder_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'closest_point_finder' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${closest_point_finder_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(closest_point_finder_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${closest_point_finder_DIR}/${_extra}")
endforeach()

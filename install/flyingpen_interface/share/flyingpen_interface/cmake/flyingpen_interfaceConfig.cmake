# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_flyingpen_interface_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED flyingpen_interface_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(flyingpen_interface_FOUND FALSE)
  elseif(NOT flyingpen_interface_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(flyingpen_interface_FOUND FALSE)
  endif()
  return()
endif()
set(_flyingpen_interface_CONFIG_INCLUDED TRUE)

# output package information
if(NOT flyingpen_interface_FIND_QUIETLY)
  message(STATUS "Found flyingpen_interface: 0.0.0 (${flyingpen_interface_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'flyingpen_interface' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${flyingpen_interface_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(flyingpen_interface_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${flyingpen_interface_DIR}/${_extra}")
endforeach()

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-gazebo7 VERSION 7.0.0)

#============================================================================
# Find ignition-cmake
#============================================================================
# If you get an error at this line, you need to install ignition-cmake
find_package(ignition-cmake3 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
ign_configure_project(
  REPLACE_IGNITION_INCLUDE_PATH gz/sim
  VERSION_SUFFIX pre1)
set (CMAKE_CXX_STANDARD 17)

#============================================================================
# Set project-specific options
#============================================================================

option(ENABLE_PROFILER "Enable Gazebo Profiler" FALSE)

if(ENABLE_PROFILER)
  add_definitions("-DIGN_PROFILER_ENABLE=1")
else()
  add_definitions("-DIGN_PROFILER_ENABLE=0")
endif()

if (UNIX AND NOT APPLE)
  set (EXTRA_TEST_LIB_DEPS stdc++fs)
else()
  set (EXTRA_TEST_LIB_DEPS)
endif()

include(test/find_dri.cmake)
FindDRI()

option(USE_SYSTEM_PATHS_FOR_PYTHON_INSTALLATION
      "Install python modules in standard system paths in the system"
      OFF)

option(USE_DIST_PACKAGES_FOR_PYTHON
      "Use dist-packages instead of site-package to install python modules"
      OFF)

#============================================================================
# Search for project-specific dependencies
#============================================================================

# Setting this policy enables using the protobuf_MODULE_COMPATIBLE
# set command in CMake versions older than 13.13
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
# This option is needed to use the PROTOBUF_GENERATE_CPP
# in case protobuf is found with the CMake config files
# It needs to be set before any find_package(...) call
# as protobuf could be find transitively by any dependency
set(protobuf_MODULE_COMPATIBLE TRUE)

ign_find_package(sdformat13 REQUIRED)
set(SDF_VER ${sdformat13_VERSION_MAJOR})

#--------------------------------------
# Find ignition-plugin
ign_find_package(ignition-plugin2 REQUIRED COMPONENTS loader register)
set(IGN_PLUGIN_VER ${ignition-plugin2_VERSION_MAJOR})

#--------------------------------------
# Find ignition-transport
ign_find_package(ignition-transport12 REQUIRED COMPONENTS log)
set(IGN_TRANSPORT_VER ${ignition-transport12_VERSION_MAJOR})

#--------------------------------------
# Find ignition-msgs
ign_find_package(ignition-msgs9 REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs9_VERSION_MAJOR})

#--------------------------------------
# Find ignition-common
# Always use the profiler component to get the headers, regardless of status.
ign_find_package(ignition-common5
  COMPONENTS
    profiler
    events
    av
  REQUIRED
)
set(IGN_COMMON_VER ${ignition-common5_VERSION_MAJOR})

#--------------------------------------
# Find ignition-fuel_tools
ign_find_package(ignition-fuel_tools8 REQUIRED)
set(IGN_FUEL_TOOLS_VER ${ignition-fuel_tools8_VERSION_MAJOR})

#--------------------------------------
# Find ignition-gui
ign_find_package(ignition-gui7 REQUIRED)
set(IGN_GUI_VER ${ignition-gui7_VERSION_MAJOR})
ign_find_package (Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
  PKGCONFIG "Qt5Core Qt5Quick Qt5QuickControls2")

#--------------------------------------
# Find ignition-physics
ign_find_package(ignition-physics6
  COMPONENTS
    heightmap
    mesh
    sdf
  REQUIRED
)
set(IGN_PHYSICS_VER ${ignition-physics6_VERSION_MAJOR})

#--------------------------------------
# Find ignition-sensors
ign_find_package(ignition-sensors7 REQUIRED
  # component order is important
  COMPONENTS
    # non-rendering
    air_pressure
    altimeter
    imu
    force_torque
    logical_camera
    magnetometer
    navsat

    # rendering
    rendering
    lidar
    gpu_lidar

    # cameras
    camera
    segmentation_camera
    depth_camera
    rgbd_camera
    thermal_camera
    wide_angle_camera
)
set(IGN_SENSORS_VER ${ignition-sensors7_VERSION_MAJOR})

#--------------------------------------
# Find ignition-rendering
ign_find_package(ignition-rendering7 REQUIRED)
set(IGN_RENDERING_VER ${ignition-rendering7_VERSION_MAJOR})

#--------------------------------------
# Find ignition-math
ign_find_package(ignition-math7 REQUIRED COMPONENTS eigen3)
set(IGN_MATH_VER ${ignition-math7_VERSION_MAJOR})

#--------------------------------------
# Find ignition-tools
ign_find_package(ignition-tools2
                 REQUIRED
                 PKGCONFIG "ignition-tools2")

#--------------------------------------
# Find ignition-utils
ign_find_package(ignition-utils2 REQUIRED COMPONENTS cli)
set(IGN_UTILS_VER ${ignition-utils2_VERSION_MAJOR})

#--------------------------------------
# Find protobuf
set(REQ_PROTOBUF_VER 3)
ign_find_package(IgnProtobuf
                 VERSION ${REQ_PROTOBUF_VER}
                 REQUIRED
                 COMPONENTS all
                 PRETTY Protobuf)
set(Protobuf_IMPORT_DIRS ${ignition-msgs9_INCLUDE_DIRS})

#--------------------------------------
# Find python
include(IgnPython)
find_package(PythonLibs QUIET)
if (NOT PYTHONLIBS_FOUND)
  IGN_BUILD_WARNING("Python is missing: Python interfaces are disabled.")
  message (STATUS "Searching for Python - not found.")
else()
  message (STATUS "Searching for Python - found version ${PYTHONLIBS_VERSION_STRING}.")

  set(PYBIND11_PYTHON_VERSION 3)
  find_package(Python3 QUIET COMPONENTS Interpreter Development)
  find_package(pybind11 2.2 QUIET)

  if (${pybind11_FOUND})
    message (STATUS "Searching for pybind11 - found version ${pybind11_VERSION}.")
	else()
		IGN_BUILD_WARNING("pybind11 is missing: Python interfaces are disabled.")
		message (STATUS "Searching for pybind11 - not found.")
	endif()
endif()
# Plugin install dirs
set(GZ_SIM_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins
)
set(GZ_SIM_GUI_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins/gui
)

#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS)

add_subdirectory(examples)

#============================================================================
# Create package information
#============================================================================
ign_create_packages()

if (${pybind11_FOUND})
	add_subdirectory(python)
endif()
#============================================================================
# Configure documentation
#============================================================================
configure_file(${CMAKE_SOURCE_DIR}/api.md.in ${CMAKE_BINARY_DIR}/api.md)
configure_file(${CMAKE_SOURCE_DIR}/tutorials.md.in ${CMAKE_BINARY_DIR}/tutorials.md)
configure_file(${CMAKE_SOURCE_DIR}/tools/desktop/ignition-gazebo.desktop.in ${CMAKE_BINARY_DIR}/ignition-gazebo${PROJECT_VERSION_MAJOR}.desktop)
configure_file(${CMAKE_SOURCE_DIR}/tools/desktop/ignition-gazebo.svg.in ${CMAKE_BINARY_DIR}/ignition-gazebo${PROJECT_VERSION_MAJOR}.svg)

# disable doxygen on macOS due to issues with doxygen 1.9.0
# there is an unreleased fix; revert this when 1.9.1 is released
# https://github.com/gazebosim/gz-sim/issues/520
if (NOT APPLE)
  ign_create_docs(
    API_MAINPAGE_MD "${CMAKE_BINARY_DIR}/api.md"
    TUTORIALS_MAINPAGE_MD "${CMAKE_BINARY_DIR}/tutorials.md"
    ADDITIONAL_INPUT_DIRS "${CMAKE_SOURCE_DIR}/src/systems ${CMAKE_SOURCE_DIR}/src/gui/plugins"
    IMAGE_PATH_DIRS "${CMAKE_SOURCE_DIR}/tutorials/files"
    TAGFILES
     "${IGNITION-MATH_DOXYGEN_TAGFILE} = ${IGNITION-MATH_API_URL}"
     "${IGNITION-MSGS_DOXYGEN_TAGFILE} = ${IGNITION-MSGS_API_URL}"
     "${IGNITION-PHYSICS_DOXYGEN_TAGFILE} = ${IGNITION-PHYSICS_API_URL}"
     "${IGNITION-PLUGIN_DOXYGEN_TAGFILE} = ${IGNITION-PLUGIN_API_URL}"
     "${IGNITION-TRANSPORT_DOXYGEN_TAGFILE} = ${IGNITION-TRANSPORT_API_URL}"
     "${IGNITION-SENSORS_DOXYGEN_TAGFILE} = ${IGNITION-SENSORS_API_URL}"
     "${IGNITION-COMMON_DOXYGEN_TAGFILE} = ${IGNITION-COMMON_API_URL}"
  )
endif()

if(TARGET doc)
  file(COPY ${CMAKE_SOURCE_DIR}/tutorials/files/ DESTINATION ${CMAKE_BINARY_DIR}/doxygen/html/files/)
endif()

# TICKTOCK MASTER HEADER (to bypass IGN_DESIGNATION)
# TODO(CH3): Remove on tock
configure_file(
  ${CMAKE_CURRENT_BINARY_DIR}/include/gz/${IGN_DESIGNATION}.hh
  ${CMAKE_CURRENT_BINARY_DIR}/include/gz/sim.hh
  COPYONLY)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/include/gz/sim.hh
  DESTINATION ${IGN_INCLUDE_INSTALL_DIR_FULL}/gz)

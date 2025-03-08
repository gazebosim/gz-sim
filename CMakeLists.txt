cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(gz-sim8 VERSION 8.9.0)
set (GZ_DISTRIBUTION "Harmonic")

#============================================================================
# Find gz-cmake
#============================================================================
# If you get an error at this line, you need to install gz-cmake
find_package(gz-cmake3 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

gz_configure_project(VERSION_SUFFIX)

#============================================================================
# Set project-specific options
#============================================================================

option(ENABLE_PROFILER "Enable Gazebo Profiler" FALSE)

if(ENABLE_PROFILER)
  add_definitions("-DGZ_PROFILER_ENABLE=1")
else()
  add_definitions("-DGZ_PROFILER_ENABLE=0")
endif()

if (UNIX AND NOT APPLE)
  set (EXTRA_TEST_LIB_DEPS stdc++fs backward_object)
else()
  set (EXTRA_TEST_LIB_DEPS backward_object)
endif()

# We're disabling pybind11 by default on Windows because they
# don't have active CI on them for now.
set(skip_pybind11_default_value OFF)
if (MSVC)
  set(skip_pybind11_default_value ON)
endif()

option(SKIP_PYBIND11
      "Skip generating Python bindings via pybind11"
      ${skip_pybind11_default_value})

include(test/find_dri.cmake)
FindDRI()

include(CMakeDependentOption)
cmake_dependent_option(USE_SYSTEM_PATHS_FOR_PYTHON_INSTALLATION
      "Install python modules in standard system paths in the system"
      OFF "NOT SKIP_PYBIND11" OFF)

cmake_dependent_option(USE_DIST_PACKAGES_FOR_PYTHON
      "Use dist-packages instead of site-package to install python modules"
      OFF "NOT SKIP_PYBIND11" OFF)

#============================================================================
# Search for project-specific dependencies
#============================================================================

# Setting this policy enables using the protobuf_MODULE_COMPATIBLE
# set command when cmake_minimum_required is less than 3.13
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
# This option is needed to use the PROTOBUF_GENERATE_CPP
# in case protobuf is found with the CMake config files
# It needs to be set before any find_package(...) call
# as protobuf could be find transitively by any dependency
set(protobuf_MODULE_COMPATIBLE TRUE)

gz_find_package(sdformat14 REQUIRED)
set(SDF_VER ${sdformat14_VERSION_MAJOR})

#--------------------------------------
# Find gz-plugin
gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

#--------------------------------------
# Find gz-transport
gz_find_package(gz-transport13 REQUIRED COMPONENTS log parameters)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

#--------------------------------------
# Find gz-msgs
gz_find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

#--------------------------------------
# Find gz-common
# Always use the profiler component to get the headers, regardless of status.
gz_find_package(gz-common5
  COMPONENTS
    av
    events
    graphics
    io
    profiler
  REQUIRED
)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

#--------------------------------------
# Find gz-fuel_tools
gz_find_package(gz-fuel_tools9 REQUIRED)
set(GZ_FUEL_TOOLS_VER ${gz-fuel_tools9_VERSION_MAJOR})

#--------------------------------------
# Find gz-gui
gz_find_package(gz-gui8 REQUIRED)
set(GZ_GUI_VER ${gz-gui8_VERSION_MAJOR})
gz_find_package (Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
  PKGCONFIG "Qt5Core Qt5Quick Qt5QuickControls2")

#--------------------------------------
# Find gz-physics
gz_find_package(gz-physics7
  COMPONENTS
    heightmap
    mesh
    sdf
  REQUIRED
)
set(GZ_PHYSICS_VER ${gz-physics7_VERSION_MAJOR})

#--------------------------------------
# Find gz-sensors
gz_find_package(gz-sensors8 REQUIRED VERSION 8.2.0
  # component order is important
  COMPONENTS
    # non-rendering
    air_pressure
    air_speed
    altimeter
    imu
    force_torque
    logical_camera
    magnetometer
    navsat

    # rendering
    dvl
    rendering
    lidar
    gpu_lidar

    # cameras
    camera
    boundingbox_camera
    segmentation_camera
    depth_camera
    rgbd_camera
    thermal_camera
    wide_angle_camera
)
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})

#--------------------------------------
# Find gz-rendering
gz_find_package(gz-rendering8 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

#--------------------------------------
# Find gz-math
gz_find_package(gz-math7 REQUIRED COMPONENTS eigen3)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

#--------------------------------------
# Find if gz command is available
find_program(GZ_TOOLS_PROGRAM gz)
if (GZ_TOOLS_PROGRAM)
  message (STATUS "Searching for gz program - found. CLI tests can be built.")
else()
  message (STATUS "Searching for gz program - not found. CLI tests are skipped.")
endif()
# Note that CLI files are installed regardless of whether the dependency is
# available during build time
set(GZ_TOOLS_VER 2)

#--------------------------------------
# Find gz-utils
gz_find_package(gz-utils2 REQUIRED COMPONENTS cli)
set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})

#--------------------------------------
# Find protobuf
gz_find_package(GzProtobuf
                REQUIRED
                COMPONENTS all
                PRETTY Protobuf)
set(Protobuf_IMPORT_DIRS ${gz-msgs10_INCLUDE_DIRS})

#--------------------------------------
# Find python
if (SKIP_PYBIND11)
  message(STATUS "SKIP_PYBIND11 set - disabling python bindings")
else()
  find_package(Python3 QUIET COMPONENTS Interpreter Development)
  if (NOT Python3_FOUND)
    GZ_BUILD_WARNING("Python is missing: Python interfaces are disabled.")
    message (STATUS "Searching for Python - not found.")
  else()
    message (STATUS "Searching for Python - found version ${PYTHONLIBS_VERSION_STRING}.")

    set(PYBIND11_PYTHON_VERSION 3)
    find_package(pybind11 2.9 CONFIG QUIET)

    if (pybind11_FOUND)
      message (STATUS "Searching for pybind11 - found version ${pybind11_VERSION}.")
    else()
      GZ_BUILD_WARNING("pybind11 is missing: Python interfaces are disabled.")
      message (STATUS "Searching for pybind11 - not found.")
    endif()
  endif()
endif()
# Plugin install dirs
set(GZ_SIM_PLUGIN_RELATIVE_INSTALL_DIR
  ${GZ_LIB_INSTALL_DIR}/gz-${GZ_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins
)
set(GZ_SIM_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${GZ_SIM_PLUGIN_RELATIVE_INSTALL_DIR}
)
set(GZ_SIM_GUI_PLUGIN_RELATIVE_INSTALL_DIR
  ${GZ_LIB_INSTALL_DIR}/gz-${GZ_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins/gui
)
set(GZ_SIM_GUI_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${GZ_SIM_GUI_PLUGIN_RELATIVE_INSTALL_DIR}
)

#============================================================================
# Configure the build
#============================================================================
gz_configure_build(QUIT_IF_BUILD_ERRORS)

add_subdirectory(examples)

#============================================================================
# Create package information
#============================================================================
gz_create_packages()

if (pybind11_FOUND AND NOT SKIP_PYBIND11)
	add_subdirectory(python)
endif()
#============================================================================
# Configure documentation
#============================================================================
configure_file(${CMAKE_SOURCE_DIR}/api.md.in ${CMAKE_BINARY_DIR}/api.md)
configure_file(${CMAKE_SOURCE_DIR}/tutorials.md.in ${CMAKE_BINARY_DIR}/tutorials.md)
configure_file(${CMAKE_SOURCE_DIR}/tools/desktop/gz-sim.desktop.in ${CMAKE_BINARY_DIR}/gz-sim${PROJECT_VERSION_MAJOR}.desktop)
configure_file(${CMAKE_SOURCE_DIR}/tools/desktop/gz-logo.svg.in ${CMAKE_BINARY_DIR}/gz-logo${PROJECT_VERSION_MAJOR}.svg)

# disable doxygen on macOS due to issues with doxygen 1.9.0
# there is an unreleased fix; revert this when 1.9.1 is released
# https://github.com/gazebosim/gz-sim/issues/520
if (NOT APPLE)
  gz_create_docs(
    API_MAINPAGE_MD "${CMAKE_BINARY_DIR}/api.md"
    TUTORIALS_MAINPAGE_MD "${CMAKE_BINARY_DIR}/tutorials.md"
    ADDITIONAL_INPUT_DIRS "${CMAKE_SOURCE_DIR}/src/systems ${CMAKE_SOURCE_DIR}/src/gui/plugins"
    IMAGE_PATH_DIRS "${CMAKE_SOURCE_DIR}/tutorials/files"
    TAGFILES
     "${GZ-MATH_DOXYGEN_TAGFILE} = ${GZ-MATH_API_URL}"
     "${GZ-MSGS_DOXYGEN_TAGFILE} = ${GZ-MSGS_API_URL}"
     "${GZ-PHYSICS_DOXYGEN_TAGFILE} = ${GZ-PHYSICS_API_URL}"
     "${GZ-PLUGIN_DOXYGEN_TAGFILE} = ${GZ-PLUGIN_API_URL}"
     "${GZ-TRANSPORT_DOXYGEN_TAGFILE} = ${GZ-TRANSPORT_API_URL}"
     "${GZ-SENSORS_DOXYGEN_TAGFILE} = ${GZ-SENSORS_API_URL}"
     "${GZ-COMMON_DOXYGEN_TAGFILE} = ${GZ-COMMON_API_URL}"
     "${GZ-GUI_DOXYGEN_TAGFILE} = ${GZ-GUI_API_URL}"
  )
endif()

if(TARGET doc)
  file(COPY ${CMAKE_SOURCE_DIR}/tutorials/files/ DESTINATION ${CMAKE_BINARY_DIR}/doxygen/html/files/)
endif()

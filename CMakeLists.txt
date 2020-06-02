cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-gazebo2 VERSION 2.19.0)

#============================================================================
# Find ignition-cmake
#============================================================================
# If you get an error at this line, you need to install ignition-cmake
find_package(ignition-cmake2 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
ign_configure_project()

#============================================================================
# Set project-specific options
#============================================================================

option(ENABLE_PROFILER "Enable Ignition Profiler" FALSE)

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

#============================================================================
# Search for project-specific dependencies
#============================================================================

ign_find_package(sdformat8 REQUIRED VERSION 8.8)

#--------------------------------------
# Find ignition-plugin
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

#--------------------------------------
# Find ignition-transport
ign_find_package(ignition-transport7 REQUIRED COMPONENTS log)
set(IGN_TRANSPORT_VER ${ignition-transport7_VERSION_MAJOR})

#--------------------------------------
# Find ignition-msgs
ign_find_package(ignition-msgs4 REQUIRED VERSION 4.9)
set(IGN_MSGS_VER ${ignition-msgs4_VERSION_MAJOR})

#--------------------------------------
# Find ignition-common
# Always use the profiler component to get the headers, regardless of status.
ign_find_package(ignition-common3 REQUIRED COMPONENTS profiler events av)
set(IGN_COMMON_VER ${ignition-common3_VERSION_MAJOR})

#--------------------------------------
# Find ignition-fuel_tools3
ign_find_package(ignition-fuel_tools3 REQUIRED)
set(IGN_FUEL_TOOLS_VER ${ignition-fuel_tools3_VERSION_MAJOR})

#--------------------------------------
# Find ignition-gui
ign_find_package(ignition-gui2 REQUIRED VERSION 2.1)
set(IGN_GUI_VER ${ignition-gui2_VERSION_MAJOR})
ign_find_package (Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
  PKGCONFIG "Qt5Core Qt5Quick Qt5QuickControls2")

#--------------------------------------
# Find ignition-physics
ign_find_package(ignition-physics1
  COMPONENTS
    mesh
    sdf
  REQUIRED
  VERSION 1.7)
set(IGN_PHYSICS_VER ${ignition-physics1_VERSION_MAJOR})

#--------------------------------------
# Find ignition-sensors
ign_find_package(ignition-sensors2 REQUIRED
  COMPONENTS
    rendering
    air_pressure
    altimeter
    camera
    gpu_lidar
    imu
    logical_camera
    magnetometer
    depth_camera
  VERSION 2.5
)
set(IGN_SENSORS_VER ${ignition-sensors2_VERSION_MAJOR})

#--------------------------------------
# Find ignition-rendering
ign_find_package(ignition-rendering2 REQUIRED)
set(IGN_RENDERING_VER ${ignition-rendering2_VERSION_MAJOR})

#--------------------------------------
# Find gflags
ign_find_package(gflags
    REQUIRED
    PKGCONFIG gflags)

#--------------------------------------
# Find Eigen
ign_find_package(EIGEN3 REQUIRED)

#--------------------------------------
# Find ignition-math
ign_find_package(ignition-math6 REQUIRED COMPONENTS eigen3 VERSION 6.2)
set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})

#--------------------------------------
# Find protobuf
set(REQ_PROTOBUF_VER 3)
ign_find_package(IgnProtobuf
                 VERSION ${REQ_PROTOBUF_VER}
                 REQUIRED
                 COMPONENTS all
                 PRETTY Protobuf)
set(PROTOBUF_IMPORT_DIRS ${ignition-msgs4_INCLUDE_DIRS})

# Plugin install dirs
set(IGNITION_GAZEBO_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins
)
set(IGNITION_GAZEBO_GUI_PLUGIN_INSTALL_DIR
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

#============================================================================
# Configure documentation
#============================================================================
configure_file(${CMAKE_SOURCE_DIR}/api.md.in ${CMAKE_BINARY_DIR}/api.md)
configure_file(${CMAKE_SOURCE_DIR}/tutorials.md.in ${CMAKE_BINARY_DIR}/tutorials.md)

ign_create_docs(
  API_MAINPAGE_MD "${CMAKE_BINARY_DIR}/api.md"
  TUTORIALS_MAINPAGE_MD "${CMAKE_BINARY_DIR}/tutorials.md"
  TAGFILES
   "${IGNITION-MATH_DOXYGEN_TAGFILE} = ${IGNITION-MATH_API_URL}"
   "${IGNITION-MSGS_DOXYGEN_TAGFILE} = ${IGNITION-MSGS_API_URL}"
   "${IGNITION-PHYSICS_DOXYGEN_TAGFILE} = ${IGNITION-PHYSICS_API_URL}"
   "${IGNITION-PLUGIN_DOXYGEN_TAGFILE} = ${IGNITION-PLUGIN_API_URL}"
   "${IGNITION-TRANSPORT_DOXYGEN_TAGFILE} = ${IGNITION-TRANSPORT_API_URL}"
   "${IGNITION-SENSORS_DOXYGEN_TAGFILE} = ${IGNITION-SENSORS_API_URL}"
   "${IGNITION-COMMON_DOXYGEN_TAGFILE} = ${IGNITION-COMMON_API_URL}"
)

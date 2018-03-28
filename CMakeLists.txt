cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-gazebo VERSION 0.1.0)

#============================================================================
# Find ignition-cmake
#============================================================================
# If you get an error at this line, you need to install ignition-cmake
find_package(ignition-cmake0 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
ign_configure_project()

#============================================================================
# Set project-specific options
#============================================================================


#============================================================================
# Search for project-specific dependencies
#============================================================================

find_package(SDFormat REQUIRED)
include_directories(${SDFormat_INCLUDE_DIRS})

#--------------------------------------
# Find ignition-gui
find_package(ignition-gui0 0.1 REQUIRED)
# Find Qt5
find_package (Qt5 COMPONENTS Core Widgets REQUIRED)

include_directories(${IGNITION-GUI_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
link_directories(${IGNITION-GUI_LIBRARY_DIRS})

#--------------------------------------
# Find ignition-math
set(IGN_MSGS_VER 5)
ign_find_package(ignition-math${IGN_MSGS_VER} REQUIRED)

#--------------------------------------
# Find ignition-msgs
set(IGN_MSGS_VER 2)
ign_find_package(ignition-msgs${IGN_MSGS_VER} REQUIRED)

#--------------------------------------
# Find ignition-transport
set(IGN_TRANSPORT_VER 4)
ign_find_package(ignition-transport${IGN_TRANSPORT_VER} REQUIRED)

#--------------------------------------
# Find ignition-common
set(IGN_COMMON_VER 1)
ign_find_package(ignition-common${IGN_COMMON_VER} REQUIRED)

#--------------------------------------
# Find gflags
find_library(gflags_LIBRARIES NAMES gflags)
find_path(gflags_INCLUDE_DIRS gflags/gflags.h ENV CPATH)
if (NOT gflags_LIBRARIES OR NOT gflags_INCLUDE_DIRS)
  BUILD_ERROR("Missing: GFlags (libgflags-dev)")
else()
  message (STATUS "Found GFlags")
  include_directories(${gflags_INCLUDE_DIRS})
  link_directories(${gflags_LIBRARY_DIRS})
endif()

#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS)

#============================================================================
# Create package information
#============================================================================
ign_create_packages()

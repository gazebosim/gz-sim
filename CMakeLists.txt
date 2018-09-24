cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-gazebo VERSION 0.1.0)

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


#============================================================================
# Search for project-specific dependencies
#============================================================================

ign_find_package(sdformat8 REQUIRED)
include_directories(${SDFormat_INCLUDE_DIRS})

#--------------------------------------
# Find ignition-plugin
set(IGN_PLUGIN_VER 1)
ign_find_package(ignition-plugin0 REQUIRED COMPONENTS loader register)

#--------------------------------------
# Find ignition-transport
set(IGN_TRANSPORT_VER 6)
ign_find_package(ignition-transport6 REQUIRED)

#--------------------------------------
# Find ignition-common
set(IGN_COMMON_VER 3)
ign_find_package(ignition-common3 REQUIRED)

#--------------------------------------
# Find ignition-gui
set(IGN_GUI_VER 1)
ign_find_package(ignition-gui1 REQUIRED)
find_package (Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED)

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

#============================================================================
# Configure documentation
#============================================================================
configure_file(${CMAKE_SOURCE_DIR}/api.md.in ${CMAKE_BINARY_DIR}/api.md)
configure_file(${CMAKE_SOURCE_DIR}/tutorials.md.in ${CMAKE_BINARY_DIR}/tutorials.md)

ign_create_docs(
  API_MAINPAGE_MD "${CMAKE_BINARY_DIR}/api.md"
  TUTORIALS_MAINPAGE_MD "${CMAKE_BINARY_DIR}/tutorials.md")

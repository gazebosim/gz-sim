cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-math5 VERSION 5.0.0)

#============================================================================
# Find ignition-cmake
#============================================================================
# If you get an error at this line, you need to install ignition-cmake
find_package(ignition-cmake1 1.0.0 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
ign_configure_project(VERSION_SUFFIX pre3)

#============================================================================
# Set project-specific options
#============================================================================

# ignition-math currently has no options that are unique to it


#============================================================================
# Search for project-specific dependencies
#============================================================================

########################################
# Include swig
find_package(SWIG QUIET)
if (NOT SWIG_FOUND)
  IGN_BUILD_WARNING("Swig is missing: Language interfaces are disabled.")
  message (STATUS "Searching for swig - not found.")
else()
  message (STATUS "Searching for swig - found.")
endif()

# Include other languages if swig was found
if (SWIG_FOUND)
  ########################################
  # Include ruby
  find_package(Ruby 1.9 QUIET)
  if (NOT RUBY_FOUND)
    IGN_BUILD_WARNING("Ruby is missing: Install ruby-dev to enable ruby interface to ignition math.")
    message (STATUS "Searching for Ruby - not found.")
  else()
    message (STATUS "Searching for Ruby - found.")
  endif()
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
set (IGN_PROJECT_ABOUT
  "Ignition Math is a component in Ignition Robotics, a set of libraries
  designed to rapidly develop robot and simulation applications. 
  This library defines a general set of math classes and functions with an
  eye toward simulation and robotics.

  ## Features

  1. Templated data types including multi-dimensional vectors, and rotational and position data types.
  2. Geometric shapes such as boxes, cylinders, spheres, and planes.
  3. A generic graph class with supporting algorithms including breadth and depth first sorting, and Dijkstra.
  4. Convenience classes such as Color, KMeans, PID, StopWatch, Spline, and Temperature.
  5. Helper functions that can clamp values, check for equality between floating point values, and more.
")

configure_file(${CMAKE_SOURCE_DIR}/api.md.in ${CMAKE_BINARY_DIR}/api.md)
configure_file(${CMAKE_SOURCE_DIR}/tutorials.md.in ${CMAKE_BINARY_DIR}/tutorials.md)

ign_create_docs()

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(gz-math7 VERSION 7.0.0)

#============================================================================
# Find gz-cmake
#============================================================================
# If you get an error at this line, you need to install gz-cmake
find_package(gz-cmake3 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
set (c++standard 17)
set (CMAKE_CXX_STANDARD 17)
gz_configure_project(
  REPLACE_IGNITION_INCLUDE_PATH gz/math
  VERSION_SUFFIX pre1)

#============================================================================
# Set project-specific options
#============================================================================

option(USE_SYSTEM_PATHS_FOR_RUBY_INSTALLATION
      "Install ruby modules in standard system paths in the system"
      OFF)

option(USE_SYSTEM_PATHS_FOR_PYTHON_INSTALLATION
      "Install python modules in standard system paths in the system"
      OFF)

option(USE_DIST_PACKAGES_FOR_PYTHON
      "Use dist-packages instead of site-package to install python modules"
      OFF)

#============================================================================
# Search for project-specific dependencies
#============================================================================

#--------------------------------------
# Find gz-utils
gz_find_package(gz-utils2 REQUIRED)
set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})

#--------------------------------------
# Find eigen3
gz_find_package(
  EIGEN3
  REQUIRED_BY eigen3
  PRETTY eigen3
  PURPOSE "Provide conversions to eigen3 types")

########################################
# Include swig
find_package(SWIG QUIET)
if (NOT SWIG_FOUND)
  IGN_BUILD_WARNING("Swig is missing: Language interfaces are disabled.")
  message (STATUS "Searching for swig - not found.")
else()
  message (STATUS "Searching for swig - found version ${SWIG_VERSION}.")
endif()

# Include other languages if swig was found
if (SWIG_FOUND)
  ########################################
  # Include ruby
  find_package(Ruby 1.9 QUIET)
  if (NOT RUBY_FOUND)
    IGN_BUILD_WARNING("Ruby is missing: Install ruby-dev to enable ruby interfaces.")
    message (STATUS "Searching for Ruby - not found.")
  else()
    message (STATUS "Searching for Ruby - found version ${RUBY_VERSION}.")
  endif()
endif()

########################################
# Python bindings
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

# Location of "fake install folder" used in tests
# Defined here at root scope so it is available for tests in src and test folders
set(FAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/fake/install")

#============================================================================
# Configure the build
#============================================================================
gz_configure_build(QUIT_IF_BUILD_ERRORS
  COMPONENTS eigen3)


#============================================================================
# Create package information
#============================================================================
gz_create_packages()

#============================================================================
# Configure documentation
#============================================================================
configure_file(${CMAKE_SOURCE_DIR}/api.md.in ${CMAKE_BINARY_DIR}/api.md)
configure_file(${CMAKE_SOURCE_DIR}/tutorials.md.in ${CMAKE_BINARY_DIR}/tutorials.md)

gz_create_docs(
  API_MAINPAGE_MD "${CMAKE_BINARY_DIR}/api.md"
  TUTORIALS_MAINPAGE_MD "${CMAKE_BINARY_DIR}/tutorials.md")

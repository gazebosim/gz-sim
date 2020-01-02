cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-math6 VERSION 6.4.0)

#============================================================================
# Find ignition-cmake
#============================================================================
# If you get an error at this line, you need to install ignition-cmake
find_package(ignition-cmake2 2.0.0 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
set (c++standard 17)
ign_configure_project()

#============================================================================
# Set project-specific options
#============================================================================

# ignition-math currently has no options that are unique to it


#============================================================================
# Search for project-specific dependencies
#============================================================================

#--------------------------------------
# Find eigen3
ign_find_package(
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
ign_configure_build(QUIT_IF_BUILD_ERRORS
  COMPONENTS eigen3)


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

cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#============================================================================
# Find ignition-cmake
#============================================================================
# If you get an error at this line, you need to install ignition-cmake
find_package(ignition-cmake0 REQUIRED)

#============================================================================
# Set up the project
#============================================================================
ign_configure_project(math 4.0.0 VERSION_SUFFIX pre2)


#============================================================================
# Set project-specific options
#============================================================================

# ignition-math currently has no options that are unique to it


#============================================================================
# Search for project-specific dependencies
#============================================================================

# ignition-math currently has no dependencies that are unique to it

#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS)


#============================================================================
# Create package information
#============================================================================
ign_create_packages()

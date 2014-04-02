include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

########################################
# Find Boost, if not specified manually
find_package(Boost ${MIN_BOOST_VERSION})
if (NOT Boost_FOUND)
  BUILD_ERROR ("Boost not found. Please install version "
    "${MIN_BOOST_VERSION} or higher.")
endif()

########################################
# Include man pages stuff
include (${project_cmake_dir}/Ronn2Man.cmake)
add_manpage_target()

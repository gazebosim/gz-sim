include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

# Detect the architecture
include (${project_cmake_dir}/TargetArch.cmake)
target_architecture(ARCH)
message(STATUS "Building for arch: ${ARCH}")

########################################
# Include swig
find_package(SWIG QUIET)
if (NOT SWIG_FOUND)
  BUILD_ERROR("Swig is required.")
  message (STATUS "Searching for swig - not found.")
else()
  message (STATUS "Searching for swig - found.")
endif()

########################################
# Include pythonlibs
find_package(PythonLibs QUIET)
if (NOT PYTHONLIBS_FOUND)
  BUILD_ERROR("PythonLibs is required.")
  message (STATUS "Searching for PythonLibs - not found.")
else()
  message (STATUS "Searching for PythonLibs - found.")
endif()

########################################
# Include man pages stuff
include (${project_cmake_dir}/Ronn2Man.cmake)
add_manpage_target()

#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/ 
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()

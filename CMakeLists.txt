cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (IGN_PROJECT_NAME "math")

project (ignition-${IGN_PROJECT_NAME})
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string (TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

set (PROJECT_MAJOR_VERSION 2)
set (PROJECT_MINOR_VERSION 9)
set (PROJECT_PATCH_VERSION 0)

set (PROJECT_VERSION ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION})
set (PROJECT_VERSION_FULL
  ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}.${PROJECT_PATCH_VERSION})

message (STATUS "${PROJECT_NAME} version ${PROJECT_VERSION_FULL}")

set (project_cmake_dir ${PROJECT_SOURCE_DIR}/cmake
  CACHE PATH "Location of CMake scripts")

include (${project_cmake_dir}/Utils.cmake)

########################################
# Package Creation: 
include (${project_cmake_dir}/cpack.cmake)
set (CPACK_PACKAGE_VERSION "${PROJECT_VERSION_FULL}")
set (CPACK_PACKAGE_VERSION_MAJOR "${PROJECT_MAJOR_VERSION}")
set (CPACK_PACKAGE_VERSION_MINOR "${PROJECT_MINOR_VERSION}")
set (CPACK_PACKAGE_VERSION_PATCH "${PROJECT_PATCH_VERSION}")

if (CPACK_GENERATOR)
  message(STATUS "Found CPack generators: ${CPACK_GENERATOR}")

  configure_file("${project_cmake_dir}/cpack_options.cmake.in"
    ${PROJECT_CPACK_CFG_FILE} @ONLY)

  set(CPACK_PROJECT_CONFIG_FILE ${PROJECT_CPACK_CFG_FILE})
  include (CPack)
endif()

# If we're configuring only to package source, stop here
if (PACKAGE_SOURCE_ONLY)
  message(WARNING "Configuration was done in PACKAGE_SOURCE_ONLY mode."
  "You can build a tarball (make package_source), but nothing else.")
  return()
endif()

#################################################
# Documentation:
add_subdirectory(doc)

# Configure documentation uploader
configure_file("${project_cmake_dir}/upload_doc.sh.in"
  ${CMAKE_BINARY_DIR}/upload_doc.sh @ONLY)

# If we're configuring only to build docs, stop here
if (DOC_ONLY)
  message(WARNING "Configuration was done in DOC_ONLY mode."
  " You can build documentation (make doc), but nothing else.")
  return()
endif()

enable_testing()

# Use GNUInstallDirst to get canonical paths
include(GNUInstallDirs)

# with -fPIC
if(UNIX AND NOT WIN32)
  set (CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "Install Prefix")
 find_program(CMAKE_UNAME uname /bin /usr/bin /usr/local/bin )
 if(CMAKE_UNAME)
   exec_program(uname ARGS -m OUTPUT_VARIABLE CMAKE_SYSTEM_PROCESSOR)
   set(CMAKE_SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR} CACHE INTERNAL
     "processor type (i386 and x86_64)")
   if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
     ADD_DEFINITIONS(-fPIC)
   endif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
 endif(CMAKE_UNAME)
endif()

set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

# developer's option to cache PKG_CONFIG_PATH and
# LD_LIBRARY_PATH for local installs
if(PKG_CONFIG_PATH)
  set (ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH})
endif()
if(LD_LIBRARY_PATH)
  set (ENV{LD_LIBRARY_PATH} ${LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH})
endif()


set (INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_INCLUDEDIR}/ignition") 
set (LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
set (BIN_INSTALL_DIR ${CMAKE_INSTALL_BINDIR})
    
set (USE_FULL_RPATH OFF CACHE BOOL "Set to true to enable full rpath")

if (USE_FULL_RPATH)
  # use, i.e. don't skip the full RPATH for the build tree
  set(CMAKE_SKIP_BUILD_RPATH  FALSE)
  
  # when building, don't use the install RPATH already
  # (but later on when installing)
  set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
  
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}")
  
  # add the automatically determined parts of the RPATH
  # which point to directories outside the build tree to the install RPATH
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  
  # the RPATH to be used when installing, but only if its not a system directory
  list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}" isSystemDir)
  if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}")
  endif("${isSystemDir}" STREQUAL "-1")
endif()

set (BUILD_IGNITION ON CACHE INTERNAL "Build Ignition Math" FORCE)
set (build_errors "" CACHE INTERNAL "build errors" FORCE)
set (build_warnings "" CACHE INTERNAL "build warnings" FORCE)

include (${project_cmake_dir}/DissectVersion.cmake)

message (STATUS "\n\n====== Finding 3rd Party Packages ======")
include (${project_cmake_dir}/SearchForStuff.cmake)
message (STATUS "----------------------------------------\n")

#####################################
MESSAGE(STATUS "Checking ignition build type")
# Set the default build type
if (NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING 
        "Choose the type of build, options are: Debug Release RelWithDebInfo Profile Check" FORCE)
endif (NOT CMAKE_BUILD_TYPE)
# TODO: still convert to uppercase to keep backwards compatibility with 
# uppercase old supported and deprecated modes
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

set (BUILD_TYPE_PROFILE FALSE)
set (BUILD_TYPE_RELEASE FALSE)
set (BUILD_TYPE_RELWITHDEBINFO FALSE)
set (BUILD_TYPE_DEBUG FALSE)

if ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "PROFILE")
  set (BUILD_TYPE_PROFILE TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELEASE")
  set (BUILD_TYPE_RELEASE TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELWITHDEBINFO")
  set (BUILD_TYPE_RELWITHDEBINFO TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
  set (BUILD_TYPE_DEBUG TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "COVERAGE")
  include (${project_cmake_dir}/CodeCoverage.cmake)
  set (BUILD_TYPE_DEBUG TRUE)
  SETUP_TARGET_FOR_COVERAGE(coverage ctest coverage)
else()
  build_error("CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug Release RelWithDebInfo Profile Check")
endif()

#####################################
# Handle CFlags
unset (CMAKE_C_FLAGS_ALL CACHE)
unset (CMAKE_CXX_FLAGS CACHE)

# Check if warning options are avaliable for the compiler and return WARNING_CXX_FLAGS variable
# MSVC generates tons of warnings on gtest code. 
# Recommended to use /W4 instead of /Wall
if (MSVC)
  set(WARN_LEVEL "/W4")
else()
  set(WARN_LEVEL "-Wall")
endif()

filter_valid_compiler_warnings(${WARN_LEVEL} -Wextra -Wno-long-long
  -Wno-unused-value -Wno-unused-value -Wno-unused-value -Wno-unused-value
  -Wfloat-equal -Wshadow -Winit-self -Wswitch-default
  -Wmissing-include-dirs -pedantic)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}${WARNING_CXX_FLAGS}")
if (DEFINED EXTRA_CMAKE_CXX_FLAGS)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CMAKE_CXX_FLAGS}")
endif()

#################################################
# OS Specific initialization
if (UNIX)
  ign_setup_unix()
else (WIN32)
  ign_setup_windows()
endif()

if(APPLE)
  ign_setup_apple()
endif()

#################################################
# Print warnings and errors
if ( build_warnings )
  message(STATUS "BUILD WARNINGS")
  foreach (msg ${build_warnings})
    message(STATUS ${msg})
  endforeach ()
  message(STATUS "END BUILD WARNINGS\n")
endif (build_warnings)

########### Add uninstall target ###############
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake")

if (build_errors)
  message(STATUS "BUILD ERRORS: These must be resolved before compiling.")
  foreach (msg ${build_errors})
    message(STATUS ${msg})
  endforeach ()
  message(STATUS "END BUILD ERRORS\n")
  message (FATAL_ERROR "Errors encountered in build. "
      "Please see the BUILD ERRORS above.")
else (build_errors)

  ########################################
  # Write the config.h file
  configure_file (${project_cmake_dir}/config.hh.in
    ${PROJECT_BINARY_DIR}/include/ignition/${IGN_PROJECT_NAME}/config.hh)
  ign_install_includes(
    ${IGN_PROJECT_NAME}${PROJECT_MAJOR_VERSION}/ignition/${IGN_PROJECT_NAME}
    ${PROJECT_BINARY_DIR}/include/ignition/${IGN_PROJECT_NAME}/config.hh)

  include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_BINARY_DIR}/include
  )
  link_directories(${PROJECT_BINARY_DIR}/src)

  if (DEFINED CMAKE_CXX_FLAGS)
    message (STATUS "Custom CFlags:${CMAKE_CXX_FLAGS}")
  else()
    message (STATUS "Use default CFlags")
  endif()
  message (STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
  message (STATUS "Install path: ${CMAKE_INSTALL_PREFIX}")
 

  if (BUILD_IGNITION)
    set(TEST_TYPE "UNIT")
    add_subdirectory(src)
    add_subdirectory(include)
    add_subdirectory(test)
  endif (BUILD_IGNITION)

  ########################################
  # Make the package config files
  set (pkgconfig_files ${PROJECT_NAME_LOWER})

  foreach (pkgconfig ${pkgconfig_files})
    configure_file(${CMAKE_SOURCE_DIR}/cmake/pkgconfig/${pkgconfig}.in
      ${CMAKE_CURRENT_BINARY_DIR}/cmake/pkgconfig/${pkgconfig}${PROJECT_MAJOR_VERSION}.pc @ONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/pkgconfig/${pkgconfig}${PROJECT_MAJOR_VERSION}.pc
      DESTINATION ${LIB_INSTALL_DIR}/pkgconfig COMPONENT pkgconfig)
  endforeach()

  ########################################
  # Make the cmake config files
  set(PKG_NAME ${PROJECT_NAME_UPPER})
  set(PKG_LIBRARIES ${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION})
  set(cmake_conf_file "${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}-config.cmake")
  set(cmake_conf_version_file "${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}-config-version.cmake")
  
  set(PKG_DEPENDS)

  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME_LOWER}-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_file}" @ONLY)

  # Use write_basic_package_version_file to generate a ConfigVersion file that
  # allow users of gazebo to specify the API or version to depend on
  # TODO: keep this instruction until deprecate Ubuntu/Precise and update with
  # https://github.com/Kitware/CMake/blob/v2.8.8/Modules/CMakePackageConfigHelpers.cmake 
  include(WriteBasicConfigVersionFile)
  write_basic_config_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_version_file}
    VERSION "${PROJECT_VERSION_FULL}"
    COMPATIBILITY SameMajorVersion)

  install(FILES 
    ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_file}
    ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_version_file}
    DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}/ COMPONENT cmake)

  ########################################
  # If present, load platform-specific build hooks.  This system is used,
  # for example, by the Ubuntu overlay, to
  # arrange for installation of Ubuntu-specific application-launching
  # configuration.
  if (EXISTS ${PROJECT_SOURCE_DIR}/cmake/packager-hooks/CMakeLists.txt)
    message(STATUS "Loading packager build hooks from cmake/packager-hooks")
    add_subdirectory(cmake/packager-hooks)
  endif()

  message(STATUS "Configuration successful. Type make to compile ${PROJECT_NAME_LOWER}")
endif(build_errors)

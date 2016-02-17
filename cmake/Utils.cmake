################################################################################
#APPEND_TO_CACHED_STRING(_string _cacheDesc [items...])
# Appends items to a cached list.
MACRO (APPEND_TO_CACHED_STRING _string _cacheDesc)
  FOREACH (newItem ${ARGN})
    SET (${_string} "${${_string}} ${newItem}" CACHE INTERNAL ${_cacheDesc} FORCE)
  ENDFOREACH (newItem ${ARGN})
  #STRING(STRIP ${${_string}} ${_string})
ENDMACRO (APPEND_TO_CACHED_STRING)

################################################################################
# APPEND_TO_CACHED_LIST (_list _cacheDesc [items...]
# Appends items to a cached list.
MACRO (APPEND_TO_CACHED_LIST _list _cacheDesc)
  SET (tempList ${${_list}})
  FOREACH (newItem ${ARGN})
    LIST (APPEND tempList ${newItem})
  ENDFOREACH (newItem ${newItem})
  SET (${_list} ${tempList} CACHE INTERNAL ${_cacheDesc} FORCE)
ENDMACRO(APPEND_TO_CACHED_LIST)

#################################################
# Macro to turn a list into a string (why doesn't CMake have this built-in?)
MACRO (LIST_TO_STRING _string _list)
    SET (${_string})
    FOREACH (_item ${_list})
      SET (${_string} "${${_string}} ${_item}")
    ENDFOREACH (_item)
    #STRING(STRIP ${${_string}} ${_string})
ENDMACRO (LIST_TO_STRING)

#################################################
# BUILD ERROR macro
macro (BUILD_ERROR)
  foreach (str ${ARGN})
    SET (msg "\t${str}")
    MESSAGE (STATUS ${msg})
    APPEND_TO_CACHED_LIST(build_errors "build errors" ${msg})
  endforeach ()
endmacro (BUILD_ERROR)

#################################################
# BUILD WARNING macro
macro (BUILD_WARNING)
  foreach (str ${ARGN})
    SET (msg "\t${str}" )
    MESSAGE (STATUS ${msg} )
    APPEND_TO_CACHED_LIST(build_warnings "build warning" ${msg})
  endforeach (str ${ARGN})
endmacro (BUILD_WARNING)

#################################################
macro (ign_add_library _name)
  set(LIBS_DESTINATION ${PROJECT_BINARY_DIR}/src)
  set_source_files_properties(${ARGN} PROPERTIES COMPILE_DEFINITIONS "BUILDING_DLL")
  add_library(${_name} SHARED ${ARGN})
  target_link_libraries (${_name} ${general_libraries})
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LIBS_DESTINATION})
  if (MSVC)
    set_target_properties( ${_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${LIBS_DESTINATION})
    set_target_properties( ${_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${LIBS_DESTINATION})
    set_target_properties( ${_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${LIBS_DESTINATION})
  endif ( MSVC )
endmacro ()

#################################################
macro (ign_add_static_library _name)
  add_library(${_name} STATIC ${ARGN})
  target_link_libraries (${_name} ${general_libraries})
endmacro ()

#################################################
macro (ign_add_executable _name)
  add_executable(${_name} ${ARGN})
  target_link_libraries (${_name} ${general_libraries})
endmacro ()


#################################################
macro (ign_install_includes _subdir)
  install(FILES ${ARGN}
    DESTINATION ${INCLUDE_INSTALL_DIR}/${_subdir} COMPONENT headers)
endmacro()

#################################################
macro (ign_install_library _name)
  set_target_properties(${_name} PROPERTIES SOVERSION ${PROJECT_MAJOR_VERSION} VERSION ${PROJECT_VERSION_FULL})
  install (TARGETS ${_name} DESTINATION ${LIB_INSTALL_DIR} COMPONENT shlib)
endmacro ()

#################################################
macro (ign_install_executable _name)
  set_target_properties(${_name} PROPERTIES VERSION ${PROJECT_VERSION_FULL})
  install (TARGETS ${_name} DESTINATION ${BIN_INSTALL_DIR})
  manpage(${_name} 1)
endmacro ()

#################################################
macro (ign_setup_unix)
  # USE_HOST_CFLAGS (default TRUE)
  # Will check building host machine for proper cflags 
  if(NOT DEFINED USE_HOST_CFLAGS OR USE_HOST_CFLAGS)
    message(STATUS "Enable host CFlags")
    include (${project_cmake_dir}/HostCFlags.cmake)
  endif()

  # USE_UPSTREAM_CFLAGS (default TRUE)
  # Will use predefined ignition developers cflags
  if(NOT DEFINED USE_UPSTREAM_CFLAGS OR USE_UPSTREAM_CFLAGS)
     message(STATUS "Enable upstream CFlags")
     include(${project_cmake_dir}/DefaultCFlags.cmake)
  endif()
endmacro()

#################################################
macro (ign_setup_windows)
  if(MSVC)
    add_definitions("/EHsc")
  endif()
endmacro()

#################################################
macro (ign_setup_apple)
  # NOTE MacOSX provides different system versions than CMake is parsing.
  #      The following table lists the most recent OSX versions
  #     9.x.x = Mac OSX Leopard (10.5)
  #    10.x.x = Mac OSX Snow Leopard (10.6)
  #    11.x.x = Mac OSX Lion (10.7)
  #    12.x.x = Mac OSX Mountain Lion (10.8)
  if (${CMAKE_SYSTEM_VERSION} LESS 10)
    add_definitions(-DMAC_OS_X_VERSION=1050)
  elseif (${CMAKE_SYSTEM_VERSION} GREATER 10 AND ${CMAKE_SYSTEM_VERSION} LESS 11)
    add_definitions(-DMAC_OS_X_VERSION=1060)
  elseif (${CMAKE_SYSTEM_VERSION} GREATER 11 AND ${CMAKE_SYSTEM_VERSION} LESS 12)
    add_definitions(-DMAC_OS_X_VERSION=1070)
  elseif (${CMAKE_SYSTEM_VERSION} GREATER 12 OR ${CMAKE_SYSTEM_VERSION} EQUAL 12)
    add_definitions(-DMAC_OS_X_VERSION=1080)
    # Use libc++ on Mountain Lion (10.8)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
  else ()
    add_definitions(-DMAC_OS_X_VERSION=0)
  endif ()

  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-undefined -Wl,dynamic_lookup")
endmacro()

# This should be migrated to more fine control solution based on set_property APPEND
# directories. It's present on cmake 2.8.8 while precise version is 2.8.7
link_directories(${PROJECT_BINARY_DIR}/test)
include_directories("${PROJECT_SOURCE_DIR}/test/gtest/include")

#################################################
# Enable tests compilation by default
if (NOT DEFINED ENABLE_TESTS_COMPILATION)
  set (ENABLE_TESTS_COMPILATION True)
endif()

# Define testing macros as empty and redefine them if support is found and 
# ENABLE_TESTS_COMPILATION is set to true
macro (ign_build_tests)
endmacro()

if (ENABLE_TESTS_COMPILATION)
  include (${project_cmake_dir}/TestUtils.cmake)
endif()

#################################################
# Macro to setup supported compiler warnings
# Based on work of Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST. 
include(CheckCXXCompilerFlag)

macro(filter_valid_compiler_warnings) 
  foreach(flag ${ARGN})
    CHECK_CXX_COMPILER_FLAG(${flag} R${flag})
    if(${R${flag}})
      set(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} ${flag}")
    endif()
  endforeach()
endmacro()

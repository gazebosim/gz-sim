configure_file (test_config.hh.in
  ${PROJECT_BINARY_DIR}/include/test_config.hh
)
include_directories (
  ${PROJECT_BINARY_DIR}/include
)

add_subdirectory(backward_vendor/backward-cpp)

if (TARGET Backward::Interface)
  get_target_property(_inc Backward::Interface INTERFACE_INCLUDE_DIRECTORIES)
  if (_inc)
    set_target_properties(Backward::Interface PROPERTIES
      INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_inc}")
  endif()
else()
  if (DEFINED BACKWARD_INCLUDE_DIRS)
    add_library(BackwardInterfaceLocal INTERFACE)
    target_include_directories(BackwardInterfaceLocal INTERFACE SYSTEM ${BACKWARD_INCLUDE_DIRS})
    add_library(Backward::Interface ALIAS BackwardInterfaceLocal)
  endif()
endif()

add_subdirectory(gtest_vendor)
add_subdirectory(benchmark)
add_subdirectory(integration)
add_subdirectory(performance)
add_subdirectory(plugins)
add_subdirectory(regression)
add_subdirectory(worlds)

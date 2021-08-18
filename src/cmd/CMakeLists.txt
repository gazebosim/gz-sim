# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable.
ign_get_libsources_and_unittests(sources gtest_sources)

add_library(ign STATIC ign.cc)

target_include_directories(ign PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ign PUBLIC
  ${PROJECT_LIBRARY_TARGET_NAME}
  ignition-gazebo${PROJECT_VERSION_MAJOR}-gui
)

set(server_executable ign-gazebo-server)
add_executable(${server_executable} server_main.cc)
target_link_libraries(${server_executable}
  ign
  ignition-utils${IGN_UTILS_VER}::cli
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

install(
  TARGETS
  ${server_executable}
  DESTINATION
  ${IGN_LIB_INSTALL_DIR}/ignition/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}/
)

set(gui_executable ign-gazebo-gui)
add_executable(${gui_executable} gui_main.cc)
target_link_libraries(${gui_executable}
  ign
  ignition-utils${IGN_UTILS_VER}::cli
)

install(
  TARGETS
  ${gui_executable}
  DESTINATION
  ${IGN_LIB_INSTALL_DIR}/ignition/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}/
)

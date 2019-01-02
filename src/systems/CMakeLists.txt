set (systems_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/DiffDrive.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/Null.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/SceneBroadcaster.cc
  PARENT_SCOPE
)

set (gtest_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/SceneBroadcaster_TEST.cc
  PARENT_SCOPE
)

set(IGNITION_GAZEBO_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins
)

add_subdirectory(physics)

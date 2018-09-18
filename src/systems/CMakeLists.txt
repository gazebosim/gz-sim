set (systems_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/Null.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/Physics.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/SceneBroadcaster.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/SystemPlugins.cc
  PARENT_SCOPE
)

set (gtest_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/SceneBroadcaster_TEST.cc
  PARENT_SCOPE
)

add_library(MockSystem SHARED MockSystem.cc)
set_property(TARGET MockSystem PROPERTY CXX_STANDARD 17)
target_link_libraries(MockSystem
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
)


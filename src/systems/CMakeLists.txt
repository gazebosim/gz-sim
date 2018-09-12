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

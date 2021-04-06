gz_add_system(scene-broadcaster
  SOURCES
    SceneBroadcaster.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

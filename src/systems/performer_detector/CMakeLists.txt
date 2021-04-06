gz_add_system(performer-detector
  SOURCES
  PerformerDetector.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

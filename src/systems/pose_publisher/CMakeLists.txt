gz_add_system(pose-publisher
  SOURCES
    PosePublisher.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

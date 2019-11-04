gz_add_system(log-video-recorder
  SOURCES
    LogVideoRecorder.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

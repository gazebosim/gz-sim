gz_add_system(log-video-recorder
  SOURCES
    LogVideoRecorder.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

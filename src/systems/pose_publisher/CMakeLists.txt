gz_add_system(pose-publisher
  SOURCES
    PosePublisher.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

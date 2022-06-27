gz_add_system(performer-detector
  SOURCES
  PerformerDetector.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

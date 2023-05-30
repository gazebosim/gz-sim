gz_add_system(odometry-publisher
  SOURCES
    OdometryPublisher.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

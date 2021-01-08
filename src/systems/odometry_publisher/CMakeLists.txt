gz_add_system(odometry-publisher
  SOURCES
    OdometryPublisher.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

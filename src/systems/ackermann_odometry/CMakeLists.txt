gz_add_system(ackermann-odometry
  SOURCES
    AckermannOdometry.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

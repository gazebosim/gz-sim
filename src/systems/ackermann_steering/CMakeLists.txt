gz_add_system(ackermann-steering
  SOURCES
    AckermannSteering.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

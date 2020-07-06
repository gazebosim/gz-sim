gz_add_system(velocity-control
  SOURCES
    VelocityControl.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

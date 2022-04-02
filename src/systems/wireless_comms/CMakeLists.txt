gz_add_system(wireless-comms
  SOURCES
  WirelessComms.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

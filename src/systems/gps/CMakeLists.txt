gz_add_system(gps
  SOURCES
    Gps.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  PRIVATE_LINK_LIBS
    ignition-sensors${IGN_SENSORS_VER}::gps
)


gz_add_system(comms-endpoint
  SOURCES
  CommsEndpoint.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

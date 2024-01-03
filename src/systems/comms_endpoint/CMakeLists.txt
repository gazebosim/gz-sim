gz_add_system(comms-endpoint
  SOURCES
  CommsEndpoint.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

gz_add_system(rf-comms
  SOURCES
  RFComms.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

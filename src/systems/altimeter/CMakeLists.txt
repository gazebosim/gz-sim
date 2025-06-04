gz_add_system(altimeter
  SOURCES
    Altimeter.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::altimeter
)

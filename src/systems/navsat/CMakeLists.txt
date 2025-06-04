gz_add_system(navsat
  SOURCES
    NavSat.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::navsat
)

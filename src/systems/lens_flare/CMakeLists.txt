gz_add_system(lens-flare
  SOURCES
    LensFlare.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-rendering::gz-rendering
    gz-sim-rendering
)

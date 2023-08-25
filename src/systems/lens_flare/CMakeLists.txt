gz_add_system(lens-flare
  SOURCES
    LensFlare.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
    gz-sim${PROJECT_VERSION_MAJOR}-rendering
)

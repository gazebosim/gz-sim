gz_add_system(free-space-explorer
  SOURCES
  FreeSpaceExplorer.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-math::gz-math
    gz-plugin::core
)

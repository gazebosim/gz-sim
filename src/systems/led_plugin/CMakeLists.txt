gz_add_system(led-plugin
  SOURCES
    LedPlugin.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-rendering::gz-rendering
    gz-sim-rendering
)

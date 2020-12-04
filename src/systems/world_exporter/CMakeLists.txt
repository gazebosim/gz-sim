gz_add_system(world-exporter
  SOURCES
    WorldExporter.cc
  PUBLIC_LINK_LIBS
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
)

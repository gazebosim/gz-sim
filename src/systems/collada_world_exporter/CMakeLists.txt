gz_add_system(collada-world-exporter
  SOURCES
    ColladaWorldExporter.cc
  PRIVATE_LINK_LIBS
    gz-common${GZ_COMMON_VER}::graphics
)

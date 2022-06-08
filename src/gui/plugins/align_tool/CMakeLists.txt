gz_add_gui_plugin(AlignTool
  SOURCES AlignTool.cc
  QT_HEADERS AlignTool.hh
  PUBLIC_LINK_LIBS
    gz-rendering${GZ_RENDERING_VER}::core
)

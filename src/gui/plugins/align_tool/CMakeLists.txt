gz_add_gui_plugin(AlignTool
  SOURCES AlignTool.cc
  QT_HEADERS AlignTool.hh
  PUBLIC_LINK_LIBS
    ${IGNITION-RENDERING_LIBRARIES}
)

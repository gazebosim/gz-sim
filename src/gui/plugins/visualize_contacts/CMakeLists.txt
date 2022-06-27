gz_add_gui_plugin(VisualizeContacts
  SOURCES
    VisualizeContacts.cc
  QT_HEADERS
    VisualizeContacts.hh
  PUBLIC_LINK_LIBS
    gz-rendering${GZ_RENDERING_VER}::core
)

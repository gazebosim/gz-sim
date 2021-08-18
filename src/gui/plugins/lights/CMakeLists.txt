gz_add_gui_plugin(Lights
  SOURCES Lights.cc
  QT_HEADERS Lights.hh
  PRIVATE_LINK_LIBS
    ${PROJECT_LIBRARY_TARGET_NAME}-rendering
)

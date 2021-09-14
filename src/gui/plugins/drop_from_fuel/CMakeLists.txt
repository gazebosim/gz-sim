gz_add_gui_plugin(DropFromFuel
  SOURCES
    DropFromFuel.cc
  QT_HEADERS
    DropFromFuel.hh
  PUBLIC_LINK_LIBS
    ${PROJECT_LIBRARY_TARGET_NAME}-rendering
)

gz_add_gui_plugin(Spawn
  SOURCES
    Spawn.cc
  QT_HEADERS
    Spawn.hh
  PUBLIC_LINK_LIBS
    ${PROJECT_LIBRARY_TARGET_NAME}-rendering
)

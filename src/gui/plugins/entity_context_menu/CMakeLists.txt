gz_add_gui_plugin(EntityContextMenuPlugin
  SOURCES
    EntityContextMenuPlugin.cc
  QT_HEADERS
    EntityContextMenuPlugin.hh
  PUBLIC_LINK_LIBS
    ${PROJECT_LIBRARY_TARGET_NAME}-rendering
)

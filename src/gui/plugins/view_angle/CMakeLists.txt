gz_add_gui_plugin(ViewAngle
  SOURCES ViewAngle.cc
  QT_HEADERS ViewAngle.hh
  PRIVATE_LINK_LIBS
    ${PROJECT_LIBRARY_TARGET_NAME}-rendering
)

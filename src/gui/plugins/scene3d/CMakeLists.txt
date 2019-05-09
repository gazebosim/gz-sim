gz_add_gui_plugin(Scene3D
  SOURCES Scene3D.cc
  QT_HEADERS Scene3D.hh
  PRIVATE_LINK_LIBS ${PROJECT_LIBRARY_TARGET_NAME}-rendering
)

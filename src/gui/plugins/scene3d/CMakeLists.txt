gz_add_gui_plugin(GzScene3D
  SOURCES Scene3D.cc
  QT_HEADERS Scene3D.hh
  PRIVATE_LINK_LIBS
    ${PROJECT_LIBRARY_TARGET_NAME}-rendering
    ignition-utils${IGN_UTILS_VER}::ignition-utils${IGN_UTILS_VER}
)

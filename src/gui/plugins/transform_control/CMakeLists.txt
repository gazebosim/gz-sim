gz_add_gui_plugin(TransformControl
  SOURCES TransformControl.cc
  QT_HEADERS TransformControl.hh
  PRIVATE_LINK_LIBS
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
)

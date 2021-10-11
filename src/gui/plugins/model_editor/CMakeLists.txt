gz_add_gui_plugin(ModelEditor
  SOURCES ModelEditor.cc
  QT_HEADERS ModelEditor.hh
  PUBLIC_LINK_LIBS
   ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
   ${PROJECT_LIBRARY_TARGET_NAME}-rendering
)

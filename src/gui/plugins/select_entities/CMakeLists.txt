gz_add_gui_plugin(SelectEntities
  SOURCES
    SelectEntities.cc
  QT_HEADERS
    SelectEntities.hh
  TEST_SOURCES
    # CameraControllerManager_TEST.cc
  PUBLIC_LINK_LIBS
   ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
   ${PROJECT_LIBRARY_TARGET_NAME}-rendering
)

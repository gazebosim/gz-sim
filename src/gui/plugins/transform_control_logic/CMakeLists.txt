gz_add_gui_plugin(TransformControlLogic
  SOURCES
    TransformControlLogic.cc
  QT_HEADERS
    TransformControlLogic.hh
  TEST_SOURCES
    # CameraControllerManager_TEST.cc
  PUBLIC_LINK_LIBS
   ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
   ${PROJECT_LIBRARY_TARGET_NAME}-rendering
)

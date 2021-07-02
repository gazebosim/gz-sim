gz_add_gui_plugin(VideoRecorderLogic
  SOURCES
    VideoRecorderLogic.cc
  QT_HEADERS
    VideoRecorderLogic.hh
  PUBLIC_LINK_LIBS
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
)

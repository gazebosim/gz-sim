gz_add_gui_plugin(VideoRecorder
  SOURCES
    VideoRecorder.cc
  QT_HEADERS
    VideoRecorder.hh
  PUBLIC_LINK_LIBS
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
)

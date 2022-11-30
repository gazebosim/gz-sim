gz_add_gui_plugin(VideoRecorder
  SOURCES
    VideoRecorder.cc
  QT_HEADERS
    VideoRecorder.hh
  PUBLIC_LINK_LIBS
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
)

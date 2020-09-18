gz_add_system(camera-video-recorder
  SOURCES
    CameraVideoRecorder.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
)

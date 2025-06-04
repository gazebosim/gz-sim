gz_add_system(camera-video-recorder
  SOURCES
    CameraVideoRecorder.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-rendering::gz-rendering
    gz-sim-rendering
)

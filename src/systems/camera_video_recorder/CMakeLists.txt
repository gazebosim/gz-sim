gz_add_system(camera-video-recorder
  SOURCES
    CameraVideoRecorder.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-rendering::gz-rendering
    gz-sim-rendering
)

gz_build_tests(TYPE UNIT
  SOURCES
    CameraVideoRecorder_TEST.cc
  LIB_DEPS
    gz-common::gz-common
    gz-rendering::gz-rendering
    gz-sim-rendering
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_CameraVideoRecorder_TEST)
  target_sources(UNIT_CameraVideoRecorder_TEST PRIVATE CameraVideoRecorder.cc)
  target_include_directories(UNIT_CameraVideoRecorder_TEST
    PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()

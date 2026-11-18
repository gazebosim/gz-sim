gz_add_system(performer-detector
  SOURCES
  PerformerDetector.cc
  TEST_SOURCES
    PerformerDetector_TEST.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)

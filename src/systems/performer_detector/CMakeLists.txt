gz_add_system(performer-detector
  SOURCES
  PerformerDetector.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)

gz_build_tests(TYPE UNIT
  SOURCES
    PerformerDetector_TEST.cc
  LIB_DEPS
    gz-common::gz-common
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_PerformerDetector_TEST)
  target_sources(UNIT_PerformerDetector_TEST PRIVATE PerformerDetector.cc)
  target_include_directories(UNIT_PerformerDetector_TEST
    PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()

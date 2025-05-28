gz_add_system(logicalaudiosensorplugin
  SOURCES
    LogicalAudioSensorPlugin.cc
    LogicalAudio.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-math::gz-math
    gz-msgs::gz-msgs
    gz-transport::gz-transport
)

set (gtest_sources
  LogicalAudio_TEST.cc
)

gz_build_tests(TYPE UNIT
  SOURCES
    ${gtest_sources}
  LIB_DEPS
    ${PROJECT_LIBRARY_TARGET_NAME}-logicalaudiosensorplugin-system
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

gz_add_system(logicalaudiosensorplugin
  SOURCES
    LogicalAudioSensorPlugin.cc
    LogicalAudio.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
    gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
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

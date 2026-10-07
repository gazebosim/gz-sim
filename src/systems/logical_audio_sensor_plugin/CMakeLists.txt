gz_add_system(logicalaudiosensorplugin
  SOURCES
    LogicalAudioSensorPlugin.cc
    LogicalAudio.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
)

set (gtest_sources
  LogicalAudio_TEST.cc
)

ign_build_tests(TYPE UNIT
  SOURCES
    ${gtest_sources}
  LIB_DEPS
    ${PROJECT_LIBRARY_TARGET_NAME}-logicalaudiosensorplugin-system
<<<<<<< HEAD
=======
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
>>>>>>> c46e470 (fix: Set GZ_IP on all types of tests (#4047))
)

gz_add_system(audioplayer
  SOURCES
    OpenAL.cc
    AudioPlayer.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
)

#set (gtest_sources
#  AudioPlayer_TEST.cc
#)
#
#ign_build_tests(TYPE UNIT
#  SOURCES
#    ${gtest_sources}
#  LIB_DEPS
#    ${PROJECT_LIBRARY_TARGET_NAME}-audioplayer-system
#)

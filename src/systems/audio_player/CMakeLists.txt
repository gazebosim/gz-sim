########################################
include (FindOpenAL)
if (NOT OPENAL_FOUND)
  BUILD_WARNING ("OpenAL not found, audio support will be disabled.")
  set (HAVE_OPENAL OFF CACHE BOOL "HAVE OpenAL")
else ()
  set (HAVE_OPENAL ON CACHE BOOL "HAVE OpenAL")
endif ()

gz_add_system(audioplayer
  SOURCES
    OpenAL.cc
    AudioPlayer.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    ${OPENAL_LIBRARY}
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

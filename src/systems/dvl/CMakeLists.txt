# Suppress maybe-uninitialized from gcc due to issue 3627
# https://github.com/gazebosim/gz-sim/issues/3627
gz_add_system(dvl
  SOURCES
    DopplerVelocityLogSystem.cc
  TEST_SOURCES
    DopplerVelocityLogSystem_TEST.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-rendering::gz-rendering
  PRIVATE_LINK_LIBS
    gz-sensors::dvl
    gz-sensors::rendering
  PRIVATE_COMPILE_OPTIONS
    $<$<CXX_COMPILER_ID:GNU>:-Wno-maybe-uninitialized>
)

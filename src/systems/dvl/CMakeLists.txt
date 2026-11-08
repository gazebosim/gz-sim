# Suppress maybe-uninitialized from gcc due to issue 3627
# https://github.com/gazebosim/gz-sim/issues/3627
gz_add_system(dvl
  SOURCES
    DopplerVelocityLogSystem.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-rendering::gz-rendering
  PRIVATE_LINK_LIBS
    gz-sensors::dvl
    gz-sensors::rendering
  PRIVATE_COMPILE_OPTIONS
    $<$<CXX_COMPILER_ID:GNU>:-Wno-maybe-uninitialized>
)

gz_build_tests(TYPE UNIT
  SOURCES
    DopplerVelocityLogSystem_TEST.cc
  LIB_DEPS
    gz-sensors::dvl
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_DopplerVelocityLogSystem_TEST)
  target_sources(UNIT_DopplerVelocityLogSystem_TEST PRIVATE DopplerVelocityLogSystem.cc)
  target_include_directories(UNIT_DopplerVelocityLogSystem_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()

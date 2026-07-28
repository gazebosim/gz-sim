gz_add_system(dvl
  SOURCES
    DopplerVelocityLogSystem.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-rendering::gz-rendering
  PRIVATE_LINK_LIBS
    gz-sensors::dvl
    gz-sensors::rendering
)

gz_build_tests(TYPE UNIT
  SOURCES
    DopplerVelocityLogSystem_TEST.cc
  LIB_DEPS
    gz-sensors::dvl
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

# Suppress maybe-uninitialized from gcc due to issue 3627
# https://github.com/gazebosim/gz-sim/issues/3627
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  target_compile_options(gz-sim-dvl-system PRIVATE
      -Wno-maybe-uninitialized)
endif()

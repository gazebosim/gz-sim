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

# Suppress maybe-uninitialized from gcc due to issue 3627
# https://github.com/gazebosim/gz-sim/issues/3627
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  target_compile_options(gz-sim-dvl-system PRIVATE
      -Wno-maybe-uninitialized)
endif()

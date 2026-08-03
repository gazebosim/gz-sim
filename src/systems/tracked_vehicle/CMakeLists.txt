gz_add_system(tracked-vehicle
  SOURCES
    TrackedVehicle.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-math::gz-math
    gz-transport::gz-transport
)

gz_build_tests(TYPE UNIT
  SOURCES
    TrackedVehicle_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

if (TARGET UNIT_TrackedVehicle_TEST)
  target_sources(UNIT_TrackedVehicle_TEST PRIVATE TrackedVehicle.cc)
  target_include_directories(UNIT_TrackedVehicle_TEST PRIVATE
    ${PROJECT_SOURCE_DIR}/test)
endif()
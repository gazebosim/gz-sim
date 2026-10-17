gz_add_system(imu
  SOURCES
    Imu.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::imu
)

gz_build_tests(TYPE UNIT
  SOURCES
    Imu_TEST.cc
  LIB_DEPS
    gz-sensors::imu
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

if (TARGET UNIT_Imu_TEST)
  target_sources(UNIT_Imu_TEST PRIVATE Imu.cc)
  target_include_directories(UNIT_Imu_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()

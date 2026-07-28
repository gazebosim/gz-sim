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

gz_add_system(cpu-lidar
  SOURCES
    CpuLidar.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::cpu_lidar
)

gz_build_tests(TYPE UNIT
  SOURCES
    CpuLidar_TEST.cc
  LIB_DEPS
    gz-sensors::cpu_lidar
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

if (TARGET UNIT_CpuLidar_TEST)
  target_sources(UNIT_CpuLidar_TEST PRIVATE CpuLidar.cc)
  target_include_directories(UNIT_CpuLidar_TEST PRIVATE
    ${PROJECT_SOURCE_DIR}/test)
endif()

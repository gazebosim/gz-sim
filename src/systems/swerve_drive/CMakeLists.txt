gz_add_system(swerve-drive
  SOURCES
    SwerveDrive.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

gz_build_tests(TYPE UNIT
  SOURCES
    SwerveDrive_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_SwerveDrive_TEST)
  target_sources(UNIT_SwerveDrive_TEST PRIVATE SwerveDrive.cc)
  target_include_directories(UNIT_SwerveDrive_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()

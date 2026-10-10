gz_add_system(mecanum-drive
  SOURCES
    MecanumDrive.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)

gz_build_tests(TYPE UNIT
  SOURCES
    MecanumDrive_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_MecanumDrive_TEST)
  target_sources(UNIT_MecanumDrive_TEST PRIVATE MecanumDrive.cc)
  target_include_directories(UNIT_MecanumDrive_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()

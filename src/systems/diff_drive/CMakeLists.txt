gz_add_system(diff-drive
  SOURCES
    DiffDrive.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)

gz_build_tests(TYPE UNIT
  SOURCES
    DiffDrive_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

if (TARGET UNIT_DiffDrive_TEST)
  target_sources(UNIT_DiffDrive_TEST PRIVATE DiffDrive.cc)
endif()

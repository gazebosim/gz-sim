gz_add_system(diff-drive
  SOURCES
    DiffDrive.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
<<<<<<< HEAD
=======

gz_build_tests(TYPE UNIT
  SOURCES
    DiffDrive_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_DiffDrive_TEST)
  target_sources(UNIT_DiffDrive_TEST PRIVATE DiffDrive.cc)
  target_include_directories(UNIT_DiffDrive_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()
>>>>>>> c46e470 (fix: Set GZ_IP on all types of tests (#4047))

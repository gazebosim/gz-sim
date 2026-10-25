gz_add_system(free-space-explorer
  SOURCES
  FreeSpaceExplorer.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-math::gz-math
    gz-plugin::core
)

gz_build_tests(TYPE UNIT
  SOURCES
    FreeSpaceExplorer_TEST.cc
  LIB_DEPS
    gz-common::gz-common
    gz-math::gz-math
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_FreeSpaceExplorer_TEST)
  target_sources(UNIT_FreeSpaceExplorer_TEST PRIVATE FreeSpaceExplorer.cc)
  target_include_directories(UNIT_FreeSpaceExplorer_TEST
    PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()

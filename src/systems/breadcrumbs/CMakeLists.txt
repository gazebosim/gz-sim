gz_add_system(breadcrumbs
  SOURCES
    Breadcrumbs.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-transport::gz-transport
)

gz_build_tests(TYPE UNIT
  SOURCES
    Breadcrumbs_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_Breadcrumbs_TEST)
  target_sources(UNIT_Breadcrumbs_TEST PRIVATE Breadcrumbs.cc)
  target_include_directories(UNIT_Breadcrumbs_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()

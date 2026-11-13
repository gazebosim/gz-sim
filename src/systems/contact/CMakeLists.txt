gz_add_system(contact
  SOURCES
    Contact.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
)

gz_build_tests(TYPE UNIT
  SOURCES
    Contact_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_Contact_TEST)
  target_sources(UNIT_Contact_TEST PRIVATE Contact.cc)
  target_include_directories(UNIT_Contact_TEST PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()

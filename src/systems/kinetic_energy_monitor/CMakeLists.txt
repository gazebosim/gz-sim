gz_add_system(kinetic-energy-monitor
  SOURCES
    KineticEnergyMonitor.cc
  PUBLIC_LINK_LIBS
    gz-transport::gz-transport
)

gz_build_tests(TYPE UNIT
  SOURCES
    KineticEnergyMonitor_TEST.cc
  ENVIRONMENT
    GZ_SIM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    GZ_IP=127.0.0.1
)

if (TARGET UNIT_KineticEnergyMonitor_TEST)
  target_sources(UNIT_KineticEnergyMonitor_TEST PRIVATE KineticEnergyMonitor.cc)
  target_include_directories(UNIT_KineticEnergyMonitor_TEST
    PRIVATE ${PROJECT_SOURCE_DIR}/test)
endif()

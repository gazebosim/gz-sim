gz_add_system(kinetic-energy-monitor
  SOURCES
    KineticEnergyMonitor.cc
  TEST_SOURCES
    KineticEnergyMonitor_TEST.cc
  PUBLIC_LINK_LIBS
    gz-transport::gz-transport
)

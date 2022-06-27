gz_add_system(kinetic-energy-monitor
  SOURCES
    KineticEnergyMonitor.cc
  PUBLIC_LINK_LIBS
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)

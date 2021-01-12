gz_add_system(kinetic-energy-monitor
  SOURCES
    KineticEnergyMonitor.cc
  PUBLIC_LINK_LIBS
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
)

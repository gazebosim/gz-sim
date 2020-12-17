gz_add_system(vehicle-damage
  SOURCES
    VehicleDamage.cc
  PUBLIC_LINK_LIBS
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
)

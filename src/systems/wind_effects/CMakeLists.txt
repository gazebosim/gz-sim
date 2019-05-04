gz_add_system(wind-effects
  SOURCES
    WindEffects.cc
  PUBLIC_LINK_LIBS
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    # Include ign-sensors for noise models
    ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
)


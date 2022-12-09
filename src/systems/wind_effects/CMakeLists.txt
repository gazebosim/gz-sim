gz_add_system(wind-effects
  SOURCES
    WindEffects.cc
  PUBLIC_LINK_LIBS
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
    # Include gz-sensors for noise models
    gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
)

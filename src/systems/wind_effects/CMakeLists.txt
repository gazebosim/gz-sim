gz_add_system(wind-effects
  SOURCES
    WindEffects.cc
  PUBLIC_LINK_LIBS
    gz-transport::gz-transport
    # Include gz-sensors for noise models
    gz-sensors::gz-sensors
)

gz_add_system(tracked-vehicle
  SOURCES
    TrackedVehicle.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-math::gz-math
    gz-transport::gz-transport
)

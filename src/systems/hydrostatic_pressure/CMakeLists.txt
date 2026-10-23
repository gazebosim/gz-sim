gz_add_system(hydrostatic-pressure
  SOURCES
  HydrostaticPressure.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
    gz-transport::gz-transport
)

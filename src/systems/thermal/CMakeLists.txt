gz_add_system(thermal
  SOURCES
    Thermal.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

gz_add_system(thermal-sensor
  SOURCES
    ThermalSensor.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

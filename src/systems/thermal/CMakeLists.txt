gz_add_system(thermal
  SOURCES
    Thermal.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

gz_add_system(thermal-sensor
  SOURCES
    ThermalSensor.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

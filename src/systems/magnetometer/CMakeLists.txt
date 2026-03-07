gz_add_system(magnetometer
  SOURCES
    Magnetometer.cc
    WorldMagneticModel.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::magnetometer
)

# Install WMM coefficient data file so it can be found at runtime
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/wmm/WMMHR.COF
  DESTINATION ${GZ_DATA_INSTALL_DIR}/wmm
)

gz_add_system(magnetometer
  SOURCES
    Magnetometer.cc
    WorldMagneticModel.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::magnetometer
  PRIVATE_COMPILE_DEFS
    GZ_SIM_WMM_INSTALL_PATH="${GZ_DATA_INSTALL_DIR}/wmm"
)

# Install WMM coefficient data file so it can be found at runtime
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/wmm/WMMHR.COF
  DESTINATION ${GZ_DATA_INSTALL_DIR}/wmm
)

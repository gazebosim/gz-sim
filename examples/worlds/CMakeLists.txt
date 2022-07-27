file(GLOB files "*.sdf")
install(FILES ${files}
  DESTINATION ${GZ_DATA_INSTALL_DIR}/worlds)
install(DIRECTORY dem_moon
  DESTINATION ${GZ_DATA_INSTALL_DIR}/worlds)

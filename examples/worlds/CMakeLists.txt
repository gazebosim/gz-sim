file(GLOB files "*.sdf")
install(FILES ${files}
  DESTINATION ${IGN_DATA_INSTALL_DIR}/worlds)

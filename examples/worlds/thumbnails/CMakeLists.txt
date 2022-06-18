file(GLOB files "*.png")
install(FILES ${files}
  DESTINATION ${IGN_DATA_INSTALL_DIR}/worlds/thumbnails)

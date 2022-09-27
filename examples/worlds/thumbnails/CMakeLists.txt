file(GLOB files "*.png")
install(FILES ${files}
  DESTINATION ${GZ_DATA_INSTALL_DIR}/worlds/thumbnails)

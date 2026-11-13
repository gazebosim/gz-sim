file(GLOB files "*.sdf")
install(FILES ${files}
  DESTINATION ${GZ_DATA_INSTALL_DIR}/worlds)

file(GLOB csv_files "*.csv")
install(FILES ${csv_files}
  DESTINATION ${GZ_DATA_INSTALL_DIR}/worlds)

file(GLOB urdf_files "*.urdf")
install(FILES ${urdf_files}
  DESTINATION ${GZ_DATA_INSTALL_DIR}/worlds)

add_subdirectory(thumbnails)

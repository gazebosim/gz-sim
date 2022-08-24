gz_install_all_headers()

add_subdirectory(components)

install (FILES server.config playback_server.config DESTINATION ${GZ_DATA_INSTALL_DIR})

ign_install_all_headers()

add_subdirectory(components)

install (FILES server.config DESTINATION ${IGN_DATA_INSTALL_DIR})

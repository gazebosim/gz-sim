if(ENABLE_GUI)
  gz_install_all_headers()
else()
  gz_install_all_headers(EXCLUDE_DIRS gui)
endif()

add_subdirectory(components)

install (FILES server.config playback_server.config DESTINATION ${GZ_DATA_INSTALL_DIR})

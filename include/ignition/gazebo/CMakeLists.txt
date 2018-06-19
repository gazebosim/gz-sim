ign_install_all_headers(
  EXCLUDE_FILES
  ComponentFactory.hh)

install (FILES gui.config DESTINATION
  ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/ignition/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}/)

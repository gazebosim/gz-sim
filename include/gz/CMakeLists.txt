add_subdirectory(sim)
# Manually install the redirection header sim.hh
# Do not merge this forward to gz-sim7
install(FILES sim.hh DESTINATION ${IGN_INCLUDE_INSTALL_DIR_FULL}/gz)

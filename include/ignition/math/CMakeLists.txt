# Exclude the detail directory from inclusion. The purpose is to prevent the detail/* header files from being included in math.hh. A side effect is that the detail headers are not installed. The next install line solves this problem.
ign_install_all_headers(EXCLUDE_DIRS detail)
install(DIRECTORY detail DESTINATION ${IGN_INCLUDE_INSTALL_DIR_FULL}/ignition/${IGN_DESIGNATION})

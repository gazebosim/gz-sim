# The headers in the python directory contain pybind11 headers. We don't want these included in gz/sim.hh because
# 1. Python is optional
# 2. There are header name conflicts with Qt
# We still want to install the headers so that users that enable python can still include them.
set(excluded_dirs python)
if(NOT ENABLE_GUI)
  list(APPEND excluded_dirs "gui")
endif()

gz_install_all_headers(EXCLUDE_DIRS ${excluded_dirs})

# Only install these if pybind11 was found. Otherwise the headers would
# declare ComponentPybindRegistry while src/ComponentPybindRegistry.cc was
# never compiled into the library, so a downstream consumer that does have
# pybind11 would fail to link against symbols that are not there.
if (pybind11_FOUND)
  install(
    DIRECTORY python
    DESTINATION "${GZ_INCLUDE_INSTALL_DIR_FULL}/${PROJECT_INCLUDE_DIR}"
    COMPONENT headers
    FILES_MATCHING PATTERN "*.hh")
endif()

add_subdirectory(components)

install (FILES server.config playback_server.config DESTINATION ${GZ_DATA_INSTALL_DIR})
